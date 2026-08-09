# esp32_camera_utils — Review & Enhancement Plan

> Review against `.ai/instructions.md` (SINGLE SOURCE OF TRUTH), `.ai/instructions.yaml`,
> `.greptile/rules.md`, and prior review plans for `tflite_micro_helper` and
> `meter_reader_tflite`. Status: **Active** (used by all readers).

Scope: all 24 files in `components/esp32_camera_utils`:
`esp32_camera_utils.{h,cpp}`, `image_processor.{h,cpp}`, `camera_window_control.{h,cpp}`,
`crop_zone_handler.{h,cpp}`, `buffer_pool.{h,cpp}`, `scaler.{h,cpp}`, `cropper.{h,cpp}`,
`rotator.{h,cpp}`, `drawing_utils.{h,cpp}`, `preview_web_handler.{h,cpp}`,
`debug_utils.h`, `__init__.py`, `README.md`, `manifest.json`, `esp32_camera_utils_controls.yaml`.

---

## 1. Summary

`esp32_camera_utils` is the shared camera/image core. It is well-factored: the
`TrackedBuffer` RAII pattern with public-pool + direct-PSRAM allocation, static `BufferPool`
with `active_instances` leak counting, overflow-checked JPEG decode (`get_jpeg_dimensions`
marker walk + `uint64` size guard), consolidated `normalize_rotation()` (removed 5 copies),
modular `Scaler`/`Cropper`/`Rotator` with §8.1 overflow guards, and good dual-target tests.
The findings below are **hardening + one correctness inconsistency + coverage gaps** — no
memory-leak blocker found in the primary paths.

---

## 2. Findings (priority per instructions.md §11)

### 🔴 BLOCKER

B1. **Channel-order inconsistency between the two JPEG paths (BGR vs RGB).**
    - `split_image_in_zone()` (decode-once/master path) explicitly does a **BGR→RGB swap**
      when `master_channels==3 && model_channels>=3`, then calls
      `process_rgb888_crop_and_scale_to_*` which reads `src_pos..+2` as **R,G,B** (RGB order).
    - `process_jpeg_zone_to_buffer()` (single-zone path) calls the *same*
      `process_rgb888_crop_and_scale_to_*` **directly on the decoder buffer without any swap**,
      reading the decoder output as RGB. But the grayscale-conversion branch just above reads
      the same decoder output as **BGR** (`b=src[idx]; g=src[idx+1]; r=src[idx+2]`, then
      luminance from r,g,b).
    - The decoder byte order is ambiguous: the code comments record it has been *toggled*
      ("Decoder now outputs RGB correctly, so this swap breaks it" vs the old BGR behavior).
      So one of the two JPEG paths feeds the model the wrong channel order, silently degrading
      accuracy (RGB models only; grayscale models are order-independent).
    **Action:** E2 — pin a single decoder byte-order contract and apply the BGR→RGB swap (or
    the `input_order` mapping) consistently in ONE place (both paths), plus a test with a known
    RGB-tagged image.

### 🟠 WARNING

W1. **`adjust_zone_for_jpeg` MCU-alignment can shift a zone's *top-left* corner.**
    It recomputes `adjusted.x1 = zone.x1 + (width - adjusted_width)/2` etc., so for an
    already-aligned zone the `(width+4)/8*8` then `-=8` logic can **shrink** a zone that's not
    a multiple of 8 and recenter it — *after* bounds-clamping. When `adjusted.x2` is re-clamped
    to `max_width`, `final_width=(final/8)*8` and `x2=x1+final_width` can still exceed the
    original `x2` (round-up), reading up to 7 px past the zone's right edge (still within the
    image if `x2≤max_width`). Zone math is not guaranteed-monotone: callers trust
    `zone.x1→x2` stays within crop. **Action:** E1 — make it monotone & bounds-conservative.

W2. **`BufferPool::acquire` oversize reuse uses a float `size * 1.2f` threshold.**
    `slot.size <= size * OVERSIZE_THRESHOLD_FACTOR` is `size_t * float` — fine at these sizes,
    but `1.2f` is a magic number and a non-integral comparison invites subtle off-by-overhead
    reuse (a 50KB request can grab a 60KB slot). Acceptable; document and make it `constexpr`
    integer math. INFO/low (see E5).

W3. **`TrackedBuffer`/pool lifecycle relies on `is_pooled` + `pool_` reset ordering.**
    `unload`/`reinitialize` recreates `ImageProcessor`, and `last_processed_image_` (a
    `RotatedPreviewImage`) may **outlive** its owning `ImageProcessor`'s pool if a preview image
    is still referenced after re-init — the `TrackedBuffer` destructor calls
    `buffer_pool_.release()` on a **static** pool (good, outlives instances) but a pooled
    buffer returned with capacity `size(sz)` can be handed to a *different* size later. The
    static `active_instances` counter is used for leak detection but **nobody logs/asserts** it
    crossing zero unexpectedly. **Action:** E3 — assert/log pool reciprocity and document the
    static-pool sharing semantics.

W4. **`get_required_buffer_size()` and the crop functions trust `model_width*height*channels`
    fit `size_t`; model dimension multiplication is checked (uint64) but `channels` can be >3.**
    `arrange_channels` handles `channels>=3` and `channels==1` but not `channels==2` (RGB565
    path uses 2-byte, but the float32/uint8 RGB565 crop feeds `arrange_channels` with
    `output_channels` derived from `model_channels`; a `model_channels==2` model yields
    writes at `dst_pos+1` only). Schema allows 1–4. `channels==2` is underhandled. **Action:** E4.

W5. **`Esp32CameraUtils` has *two* coordinate spaces for `process_zone`.** It translates global
    zones to local window coords and clips; but `meter_reader_tflite`'s `CameraCoordinator`
    calls `ImageProcessor::split_image_in_zone` with zones that assume *already-rotated/expected*
    coords (the windowing/rotation interplay is documented as a limitation, §3.2 warning).
    A mismatch can silently crop the wrong region. **Action:** E6 — centralize/ document the
    coordinate-space contract (see Do NOT change).

W6. **`test_camera_after_reset()` in `Esp32CameraUtils` is a stub** that "just returns true".**
    `meter_reader_tflite` has its own version; neither actually exercises the camera. The §3.2
    "camera recovery after N≥3 timeouts, esp_restart() after 5+" guidance is **not implemented**
    here. **Action:** E7 — implement a bounded recovery (N-frame timeout counter + optional
    `esp_restart()`), or remove the stub and rely on the FrameState watchdog in the reader.

### 🟡 INFO

I1. **Dead/legacy code:** `apply_software_rotation()` (duplicate of `Rotator::perform_rotation`
    under `#else`), `CameraCoordinator::test_camera_after_reset` sibling, and the commented-out
    BGR—RGB swap blocks (kept as documentation). Prefer deleting the commented-out branches.
I2. **`crop_zone_handler.parse_zones` manual JSON parser:** no validation that `x1<x2` /
    `y1<y2`, and silently accepts the first 4 ints. A malformed zone flows to processing.
    Bounds-check at parse (low risk, but cheap).
I3. **`DrawingUtils`/`camera_window_control` sensor math:** OV2640 `(w/4)*4` alignment and the
    OV3660/OV5640 `set_res_raw` calls are sensor-specific and not unit-tested; `get_framesize_from_dimensions`
    returns `FRAMESIZE_UXGA` default fallback even for >1920×1080 — fine.
I4. **`BufferPool` is a file-static singleton shared across ImageProcessor instances** (static
    member). With multiple readers (cold/hot meter models), buffers are pooled across them —
    acceptable, but document the shared-pool threading (mutex-guarded, so race-safe).
I5. **`image_processor` uses `new TrackedBuffer`/`new RotatedPreviewImage`** (9 new-sites).
    All immediately wrapped in `unique_ptr`/`shared_ptr` → RAII is satisfied, but the manual
    `new` is a §3.1 "no manual new/delete" nit; prefer `std::make_unique`/`std::make_shared`
    (still requires C++ `enable_from_this`-free ownership transfer — fine here).
I6. **`rotation` is `float` but schema says "arbitrary" while `__init__` maps only
    0/90/180/270** (`ROTATION_OPTIONS` unused — dead map). The C++ supports arbitrary fine angles
    via software rotator; document and remove the unused map.
I7. **`debug_utils.h` `ScopedDuration` is always-on** (starts `millis()` even when no
    `DEBUG_*`), adding a per-call `millis()` cost in `crop_zone_handler` parse; the
    `rotator.h` version is correctly no-op when debug off. **Action:** gate `ScopedDuration`
    like `rotator.h` (zero-cost when not debug).

---

## 3. Enhancement Plan (ordered)

### E1 — Make `adjust_zone_for_jpeg` monotone & conservative 🟠
**What:** rework so `x1` is non-decreasing, `x2=y`-clamped, and final `x2 = min(zone.x2, max_width)`
after 8-alignment (never round up past the requested zone). Add validation.
**Why:** W1 — current code can shrink/recenter and round up past the caller's zone.

### E2 — Single decoder byte-order contract (BGR vs RGB) 🔴 ✅ DONE
<br>Fixed: added `ensure_rgb_order()` (canonical RGB) helper; `process_jpeg_zone_to_buffer`
<br>now normalizes BGR→RGB for RGB models; master path uses the same shared helper.
**What:** decide one canonical byte order for `decode_jpeg` output (recommend **RGB**) and apply
the BGR→RGB swap + `input_order` mapping in a single helper used by `split_image_in_zone` AND
`process_jpeg_zone_to_buffer`; update the grayscale branch and remove the ambiguity comments.
**Why:** B1 — one of the two JPEG paths feeds wrong channel order to RGB models.
**How:** factor `ensure_rgb_order(decoded_ptr, w*h)`; call right after `decode_jpeg` in both
paths; keep `input_order`-based BGR remap in `arrange_channels` only (drop the master-path swap,
or drop the per-zone read-order assumption — pick one). Add a regression test with a known
RGB image (and a `debug`-path log asserting channel order).

### E3 — Pool lifecycle: log/assert `active_instances` and document static-pool sharing 🟠
**What:** after `reinitialize_image_processor`/`unload`, log residual `TrackedBuffer::active_instances`
and assert it returns to 0; document that `last_processed_image_` may outlive the ImageProcessor
and that the **static** `buffer_pool_` is shared across instances (thread-safe).
**Why:** W3 — pooled buffers returned with capacity can be re-homed; `active_instances` is a
leak detector that is currently unread.

### E4 — Handle `model_channels==2` in the crop/arrange path 🟠
**What:** make `arrange_channels` (both float/uint8) branch on `output_channels==2` (write
GRAY/paired layout or clamp), and document the 1–4 range.
**Why:** W4 — schema allows 1–4 but the 2-channel case is underhandled.

### E5 — Replace magic `1.2f` oversize reuse with `constexpr` integer math 🟡
**What:** `oversize_ok(size, cap)` using `cap <= size + size/5` (no float), named constant.
**Why:** W2 — removes magic float threshold, avoids float-on-size arithmetic.

### E6 — Coordinate-space contract for windowed zones 🟡
**What:** document + centralize the mapping between global crop zones, window offsets, rotation,
and processor-local coords; add a clear `cv.Invalid` when the `camera_window` config would make
zones fall outside the active window (§3.3 bounds-check).
**Why:** W5 — the window/rotation interplay is easy to misuse.

### E7 — Implement or remove camera-recovery stub 🟡
**What:** either implement `Esp32CameraUtils` frame-timeout recovery (track N≥3 consecutive timeouts
on the FrameState watchdog, call `basic_camera_recovery()`, `esp_restart()` at 5+) or delete the
stub and rely on `meter_reader_tflite`'s watchdog (which exists in `loop()`).
**Why:** W6 / §3.2 guidance.

### E8 — Remove unused `ROTATION_OPTIONS` map + dead `apply_software_rotation` 🟡
**What:** delete the unused `ROTATION_OPTIONS` dict in `__init__.py`, and the
`#else` fallback `apply_software_rotation()` duplicate (which duplicates `Rotator`).
**Why:** I6/I1 — dead/side-by-side code.

### E9 — Gate `ScopedDuration` (zero-cost when not debug) 🟡 ✅ DONE
<br>Fixed: `debug_utils.h` now provides a zero-cost no-op `ScopedDuration` when
<br>`DEBUG_DURATION` is not defined (mirrors `rotator.h`'s `ScopedTimer`).
**What:** mirror `rotator.h`'s `#ifdef DEBUG_*` no-op pattern in `debug_utils.h`.
**Why:** I7 — `crop_zone_handler` parse calls it unconditionally; saves a `millis()` when no debug.

### E10 — Harden `parse_zones` (validate `x1<x2`, `y1<y2`, cap zone count) 🟡 ✅ DONE
<br>Fixed: `parse_zones` now rejects invalid zones (`x1>=x2` or `y1>=y2`) and caps
<br>the count at `MAX_CROP_ZONES` (16), matching `StaticVector<float,16>` in readers.
**What:** bounds-check each zone at parse and reject invalid ones; cap at 16 (matching
`StaticVector<float,16>` in readers).
**Why:** I2 — malformed zones currently flow through silently.

### E11 — RAII style: `make_unique`/`make_shared` for the `new TrackedBuffer/RotatedPreviewImage` 🟡
**What:** replace `new` sites with `std::make_unique`+`std::make_shared` where ownership transfer
allows.
**Why:** I5 / §3.1 nit (mechanical).

### E12 — Expand tests to the RGB/grayscale/channel-order + recovery paths 🟡
**What:** add an `esp32-s3.yaml` case with `debug`+`rotation:90`+`camera_window`+`scaler/cropper`
(already present) AND a JPEG **RGB model** end-to-end that asserts the model receives RGB
(see E2), plus a `process_zone` (windows) case. Keep dual-target §9.1 coverage.
**Why:** the esp32.yaml/common.yaml are minimal (no window/rotation/scaler); the S3 test is the
only one exercising these paths — the decoder channel-order bug (E2) is otherwise untested.

---

## 4. Sequencing & review priorities
1. **E2 (B1, channel-order)** — correctness first; it silently degrades RGB-model accuracy.
2. **E1 (zone alignment), E3 (pool lifecycle), E4 (channels==2)** — robustness.
3. **E7 (recovery)**, then the INFO cleanups (E5–E12) in any order (low risk).

## 5. Do NOT change
- The **16-byte / 64-byte alignment** on JPEG (`jpeg_calloc_align(16)`) and model-input
  (`heap_caps_aligned_alloc(64,…)`) buffers (§3.2/§3.3 — required by TFLite Micro & JPEG libs).
- The PSRAM-preferred, internal-fallback allocation & the `TrackedBuffer`/`UniqueBufferPtr` RAII
  ownership contract (release-order-safety), and the **static** `buffer_pool_` sharing semantics.
- `normalize_rotation()` consolidation and the `Rotator` overflow guards (§8.1) — keep.
- The span-based `split_image_in_zone()` C++20 overload (safer than the legacy copy).
- The dual-target test structure (`esp32.yaml` + `esp32-s3.yaml`).
