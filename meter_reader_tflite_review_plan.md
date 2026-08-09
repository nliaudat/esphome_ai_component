# meter_reader_tflite — Review & Enhancement Plan

> Review conducted against `.ai/instructions.md` (SINGLE SOURCE OF TRUTH),
> `.ai/instructions.yaml`, `.greptile/rules.md`, and the prior
> `tflite_micro_helper_review_plan.md`. Status: **Active** (v2.0.0 per manifest.json).

Scope: all 15 files in `components/meter_reader_tflite`:
`meter_reader_tflite.{h,cpp}`, `tflite_coordinator.{h,cpp}`, `camera_coordinator.{h,cpp}`,
`value_validator_coordinator.{h,cpp}`, `flashlight_coordinator.{h,cpp}`,
`debug_coordinator.{h,cpp}`, `__init__.py`, `README.md`, `manifest.json`, `debug.jpg`,
`meter_reader_tflite_controls.yaml`.

---

## 1. Summary

`meter_reader_tflite` has already been refactored well: a single `std::atomic<FrameState>`
state machine (→ no TOCTOU for frame handling), a shared `publish_inference_result()` (→ no
sync/async duplication, §3.6), a consolidated `refresh_image_processor_config()` (removed the
4 old copies), coordinator separation (`TFLiteCoordinator` / `CameraCoordinator` /
`FlashlightCoordinator` / `ValueValidatorCoordinator` / `DebugCoordinator`), dual-core
double-buffering with object pools, and the new ID-reference to `tflite_micro_helper`. The
remaining work is **hardening the dual-core unload/reload ownership, removing dead code, and
fixing one broken test**, plus a few memory/correctness nits.

---

## 2. Findings (priority per instructions.md §11)

### 🔴 BLOCKER

B1. **Broken `esp32.yaml` test: `tflite_micro_helper` entry lacks required keys.**
    `tests/components/meter_reader_tflite/esp32.yaml` declares:
    ```yaml
    tflite_micro_helper:
      id: test_tflite        # no model_type, no model, no GenerateID schema fields
    ```
    `tflite_micro_helper` schema requires `model_type` (`cv.enum`) and `model`
    (`cv.Required`). This test config **fails to validate**, so `test.esp32-idf.yaml` for
    meter_reader_tflite cannot build. Meanwhile `common.yaml` / `esp32-s3.yaml` use the new
    ID-reference form correctly. The legacy `model: "file.tflite"` path is only exercised by
    `esp32.yaml` — which is currently broken — so **the legacy path is effectively untested**.
    **Action:** E12: fix `esp32.yaml` (give the helper a valid `model_type`/`model`, or
    split into a legacy-only test that omits `tflite_micro_helper`).

### 🟠 WARNING

W1. **Dual-core in-flight inference pool leak on unload/reload.**
    `inference_task` allocates `InferenceResult` from the pool *before* sending to the
    output queue. `stop_inference_pipeline()` waits for `eTaskGetState()==eDeleted` (2s
    timeout) then **force-deletes** the task (`vTaskDelete`) while it may be mid-inference
    and *outside* `xQueueReceive` — so a locally-held `InferenceResultPtr res` (and the
    `InferenceJobPtr` being processed) is **never returned to the pool / heap-freed**.
    `unload_resources()` then resets the pool arrays; repeated unload/reload cycles lose
    pool slot accounting (or leak heap-fallback jobs). The `is_inferencing_` atomic is set
    but **never consulted** by `stop_inference_pipeline`, so the sync flag is ineffective.
    **Action:** E1 — make `stop_inference_pipeline` wait on `is_inferencing_` (or use a
    completion signal) before force-delete; drain/return any in-flight pooled objects.

W2. **`is_inferencing_` is a write-only synchronization hint.** Set true before the queue
    receive and false after; `stop_inference_pipeline()` ignores it. Per §3.1's "single
    atomic state machine" rule this is an incomplete protocol. **Action:** fold into E1
    (gate unload on `is_inferencing_`, or remove the flag and rely on the eDeleted wait +
    in-flight drain).

W3. **`get_model_spec()->input_type` heuristic is inherited into processor config.**
    `refresh_image_processor_config` maps `(spec.input_type == 1) ? 0 : 1` (FLOAT↔
    UINT8 ImageProcessorInputType). This depends on `tflite_micro_helper`, where
    `get_model_spec()` derives `input_type` from a fragile `bytes == num_elements*4` guess
    (see `tflite_micro_helper_review_plan.md` W3/E5). A float model whose `bytes` happen to
    equal `num_elements*4` mis-maps and passes the wrong processor input type to the camera
    pipeline. **Action:** E2 — fix the helper (`input->type`) so the consumer mapping is
    correct, and document the mapping.

W4. **`request_preview_` is a plain non-atomic `bool` written by service/button callbacks
    and read in the main-loop inference path** (`process_full_image`, `update`, `loop`).
    ESPHome service/button handlers run in the main loop, so this is *usually* single-
    threaded — but `take_preview_image()`/`capture_preview()` may be reached from a
    different context (e.g. `defer`/async web handler). **Action:** E3 — make
    `request_preview_` `std::atomic<bool>` (like `pause_processing_`, `frame_state_`).

W5. **Legacy `model: "file.tflite"` path (deprecated) is duplicated and drift-prone.**
    `to_code()` re-implements the whole `.txt` parse + filename heuristics + CRC + PROGMEM +
    `MAX_OPERATORS` that `tflite_micro_helper/__init__.py` already owns. It also adds the
    **global** `-DMAX_OPERATORS` flag (same multi-model race as in the helper review W5/E7).
    This is the *second* copy of the `parse_model_txt_file()` logic in the repo.
    **Action:** E4 — reduce the legacy path to a thin adapter (delegate to
    `tflite_micro_helper` at codegen), or keep it but share the parse + `MAX_OPERATORS`
    max-flag logic.

W6. **`serialize_inference_metadata` builds JSON via `std::string` with `reserve(512)`**
    (§4.3 prefers pre-allocated buffers). It only runs on a *collection trigger* (rare), not
    every inference, so the hot-path impact is low — but the zone box is read from
    `crop_zone_handler_.get_zones()` into a heap `std::vector` copy. **Action** (low):
    E5 — reserve on the `zones` local or index directly to avoid the copy.

### 🟡 INFO

I1. **Dead code: `validate_and_update_reading(float raw, float conf, float &val)`** overload
    is defined+declared but **never called** — the pipeline only uses the `StaticVector`
    overload. The float overload and `set_last_valid_value(float)` legacy helpers are
    dead. **Action:** E6 — delete or document as legacy API.

I2. **`combine_readings` builds `digit_string` with `+= std::to_string(digit)` in the hot
    path** (§4.3). Minor per-digit heap allocs; a fixed char buffer + manual append avoids
    it. **Action:** E7 — use a local `char buf[16]` + `snprintf`.

I3. **`strtof` reading reconstruction (legacy `publish_inference_result` path):** `050` →
    50.0 (right for a 3‑digit meter) but loses leading‑zero semantics and `float` precision
    beyond ~7 digits. Acceptable legacy; document the digit-count limit. INFO.

I4. **`StaticVector<float,16>` caps readings at 16 digits/zones, but zone count is not
    validated against 16** anywhere in the schema or code. A 17‑zone config silently
    truncates. **Action:** E8 — validate `crop_zones`/`ImageProcessor` zone count ≤ 16 at
    config time.

I5. **`CameraCoordinator::test_camera_after_reset` is legacy, no longer called** (the
    header comment admits it). The component uses `FrameState` instead. `basic_recovery()`
    is mostly a log stub. **Action:** E6 — remove/flag dead code.

I6. **`camera_coordinator.cpp` uses `static_cast<int>` in a `switch` for rotation** — fine
    (§3.1 no C‑style casts: it's `static_cast`, good). Note `std::abs(float)` used in
    `process_full_image` preview dims — correct. No C‑style casts found in this component's
    C++ (unlike `debug_utils.cpp`). OK.

I7. **Dual-core: `process_full_image`'s sync `total_inference_time`/`capture_to_publish`
    blocks are only reachable when `SUPPORT_DOUBLE_BUFFERING` is undefined** — on dual-core
    the async path returns early. The `#ifdef SUPPORT_DOUBLE_BUFFERING{} #endif` + sync code
    structure means on single-core the sync path runs (good); on dual-core the sync timing
    block is effectively dead but harmless. Document that timing sensors only reflect the
    active path. INFO.

I8. **`set_camera` uses `static_cast<esp32_camera::ESP32Camera*>(camera)`** — on host
    (`USE_HOST_MOCK_CAMERA`) `set_camera` isn't called, so no UB; but the cast is
    unchecked if a non-ESP32Camera is ever passed. INFO.

I9. **`get_setup_priority()=LATE` + `set_timeout(MODEL_LOAD_DELAY_MS)` (10s) inside
    `setup()`:** model loads 10s after boot. On the classic ESP32 the `tflite_coord_.load_model()`
    can still exceed the 10s budget for large models — feeds `App.feed_wdt()` before load.
    Acceptable, but the 10s delay means **the meter is read-only after 10s**; document for
    the <2700ms full-shoot budget (§4). INFO.

I10. **`dump_config` lacks camera/window/rotation info** (incomplete diagnostics). INFO.

---

## 3. Enhancement Plan (ordered)

### E1 — Race-free stop/unload of the inference task (fix W1/W2) 🟠 ✅ DONE
**What:** make `stop_inference_pipeline()` (and `unload_resources()`) wait for in-flight
work to finish and return pooled objects before deleting pools.
**Why:** W1/W2 — repeated unload/reload leaks pooled `InferenceResult`/`InferenceJob`
entries and the TFLite arena is freed while the task may still be mid-`run_inference`.
**How:**
- Gate `stop_inference_pipeline()` on `is_inferencing_`: set `task_running_=false`, then
  poll `is_inferencing_` + `eTaskGetState()!=eDeleted` for up to 2s; only `vTaskDelete`
  if still stuck (and drain that task's owned pointers if it was force-deleted mid-job).
- Before `unload_resources()` resets the pools, call `stop_inference_pipeline()` *and*
  drain the output queue (already done) plus mark any force-deleted task's owned
  `InferenceResultPtr`/`InferenceJobPtr` as freed in the pool.
- Add a pool "busy-slot steal" scan in `free_inference_job`/`free_inference_result` so a
  force-deleted leak can be reclaimed by the next allocator (defensive).

### E2 — Fix the `input_type` mapping at the source (helper E5) 🟠 ✅ DONE
**What:** apply `tflite_micro_helper_review_plan.md` E5 (derive from `input->type`), then
keep the consumer mapping `(spec.input_type==1)?0:1` and document it next to
`refresh_image_processor_config()`.
**Why:** W3 — a float model currently can be mis-mapped to UINT8 processor config.
**How:** land the helper fix first (its E5), add a static_comment in
`refresh_image_processor_config` explaining FLOAT↔UINT8 mapping, and add an
`esp32-s3`/`esp32` float-model test to catch regressions.

### E3 — Make `request_preview_` atomic 🟠 ✅ DONE
**What:** `std::atomic<bool> request_preview_{false}`; use `.store()/.load()`/`exchange`.
**Why:** W4 — plain bool written by callbacks, read on main-loop inference path.
**How:** mechanical; aligns with the existing `std::atomic` style (`pause_processing_`).

### E4 — De-duplicate the legacy model path 🟠 ✅ DONE
**What:** reduce `meter_reader_tflite/__init__.py`'s legacy `model:"file.tflite"` branch to a
thin adapter that reuses `tflite_micro_helper`'s `parse_model_txt_file()`, `datasize_to_bytes`,
and a single `MAX_OPERATORS` max-flag (E7 of the helper plan) — instead of the second,
drift-prone copy.
**Why:** W5 — duplicated `.txt` parsing + global `MAX_OPERATORS` flag collision.
**How:** import the helper's `parse_model_txt_file`/`datasize_to_bytes`; keep the legacy
YAML surface working (setters) but delegate the heavy lifting; keep the deprecation print.

### E5 — Avoid the `zones` heap copy in `serialize_inference_metadata` 🟡 ✅ DONE
**What:** index `crop_zone_handler_.get_zones()` directly and drop the
`std::vector<CropZone> zones` copy; reserve `json` to a tighter bound.
**Why:** W6/§4.3 — pre-allocated buffers preferred; low impact (only on collection trigger).
**How:** iterate `const auto &zones = this->crop_zone_handler_.get_zones();` by reference.

### E6 — Remove/retire dead code 🟡 ✅ DONE
**What:** delete:
- `validate_and_update_reading(float raw, float conf, float &val)` overload (never called);
- `CameraCoordinator::test_camera_after_reset()` (legacy; header says no longer called);
- `set_last_valid_value(float)` / `set_last_valid_value(const std::string &)` if unused
  (verify callers first — they're exposed setters; keep if used as API, but they are
  dead in the .cpp).
**Why:** §11 INFO — dead code in a memory-constrained binary.
**How:** grep callers first; remove only confirmed-unused; keep header API if externally
usable (mark `[[maybe_unused]]` otherwise).

### E7 — `combine_readings` hot-path string build 🟡 ✅ DONE
**What:** use a local `char buf[16]` + `snprintf` per digit instead of
`std::string += std::to_string(digit)`.
**Why:** §4.3 — avoid heap allocs in the inference hot path (`std::string` temp in loops).
**How:** mechanical; keep the `out_str` (caller-owned) as the only dynamic string.

### E8 — Validate zone count ≤ 16 at config time 🟡 ✅ DONE
**What:** in `__init__.py`, bound `crop_zones` entries (max 16 zones) or clamp in
`crop_zone_handler_`.
**Why:** I4 — `StaticVector<float,16>` silently truncates readings if zones > 16.
**How:** add an `cv.Range`/length check on the `crop_zones`/JSON count, or a clear
`cv.Invalid`.

### E9 — Fix the broken `esp32.yaml` test 🟠 (test coverage) ✅ DONE
**What:** repair `tests/components/meter_reader_tflite/esp32.yaml` so the `tflite_micro_helper`
entry has `model_type: image` + a valid `model` (e.g. the same local
`digit_recognizer_..._uint8.tflite` used elsewhere), so `test.esp32-idf.yaml` compiles; or
target this file as a **legacy-only** test (drop the helper, keep `model:"file.tflite"`)
so the deprecated path is actually exercised and CI passes.
**Why:** B1 — the current file cannot validate/build, and the legacy path is untested.
**How:** pick the legacy-only variant (matches the file's purpose) and keep
`common.yaml`/`esp32-s3.yaml` on the new ID-reference path for dual-target §9.1 coverage.

### E10 — docs/hardening 🟡 ✅ DONE
- Document the 10s `MODEL_LOAD_DELAY_MS` and that timers only measure the *active* path
  (J8/I9).
- Tighten `dump_config` with camera/window/rotation/model state.
- Note the `strtof` digit-count/precision limit (I3).

---

## 4. Sequencing & review priorities
1. **E1 + E9 (W1/W2 + B1)** — the only correctness/CI blockers: make unload race-free and
   fix the broken test first.
2. **E2 (input_type mapping)** before any float-model enablement — land helper E5 in lockstep.
3. **E3 (atomic preview), E4 (legacy de-dup)** — robustness + drift-reduction.
4. **E5–E8 + E10 (memory/hot-path/dead-code/docs)** — low risk, incremental.

## 5. Do NOT change
- The single `std::atomic<FrameState>` frame state machine + `compare_exchange_strong`
  acquire path in `on_camera_image` (correct, satisfies §3.1/§8 TOCTOU for frames).
- The shared `publish_inference_result()` (single publishing path per §3.6).
- `refresh_image_processor_config()` consolidation (removed the 4 duplicate copies).
- The coordinator decomposition and the `TFLiteCoordinator::set_tflite()` ID-reference
  contract (backward-compatible with `meter_reader_tflite` new mode).
- The pool allocator design (RAII `unique_ptr` + custom deleters, mutex-guarded) — keep,
  but harden the stop/unload path (E1).
- Dual-core `esp_cache_msync(DIR_C2M)` flush of PSRAM crop buffers before cross-core read
  (correct & necessary on ESP32-S3).
