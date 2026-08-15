# Analog Reader — Review Plan (Bug Fixes + Enhancements)

**Scope:** `C:\dev\esphome_ai_component\components\analog_reader`
**Reviewed against:** `C:\dev\esphome_ai_component\.ai\instructions.md` (authoritative, single source of truth)
**Component status per §2.2:** **Alpha** — API may change, must stay opt-in & clearly documented.
**Role rules that apply (§3):** universal (§3.1) + Camera Consumer (§3.3) + Sensor/Output Consumer (§3.5) — `analog_reader` consumes the camera, publishes sensors, and opts into `value_validator`.

---

## 1. Context captured (files read)

| File | Role |
|------|------|
| `__init__.py` | Config schema + codegen (`USE_ANALOG_READER` define) |
| `analog_reader.{h,cpp}` | Main polling/camera-listener component |
| `multi_algorithm.cpp` | Legacy / radial / hough / template detection + preproc (CLAHE, top-hat, median) |
| `flashlight_coordinator.{h,cpp}` | Flash scheduling wrapper |
| `value_validator_coordinator.{h,cpp}` | `value_validator` opt-in wrapper |
| `README.md`, `manifest.json`, `*_algo_*.md` | Docs / algorithm notes |
| `tests/components/analog_reader/{common,esp32,esp32-s3}.yaml` | Build tests |

**Repo-wide note (§7.7 = BLOCKER):** entire repo is CRLF + heavy trailing whitespace. Not component-local; a single repo-wide LF/whitespace sweep fixes all 3 target components at once.

---

## 2. Bugs found in `analog_reader`

### B1 — Algorithm decision uses uninitialised `DetectionResult` when a path is skipped (UB)
`find_needle_angle()` builds `result_legacy/radial/hough/template` as default `DetectionResult` and only sets `has_*` flags when a branch runs. In *auto* mode:
```cpp
selected_result = result_legacy;                    // if legacy never ran -> default (angle=?, confidence=?)
if (result_radial.confidence > selected_result.confidence) selected_result = result_radial;
```
The struct has no default initialisers for `angle/confidence` (undefined values). If `algorithm == "auto"` and any branch did not run (e.g. `legacy` call skipped), comparisons read indeterminate `confidence`. The `#ifdef USE_ANALOG_READER` path default `algorithm="radial_profile"` means only that one runs — but *legacy* runs against the **raw** image while the others run on the **preprocessed** (top-hat) image; in `auto` the comparison mixes raw-vs-preprocessed scores with incomparable scales.
- **Fix (correctness):** initialise every `DetectionResult` to `{0.0f, 0.0f, ""}` (or a local `DetectionResult empty{}`), and drive selection off `has_*` flags, not off default-constructed values. Optionally guard `auto` so it only picks among branches that actually ran.

### B2 — `calibration_mapping` sorted in `setup()`, but `angle_to_value` reads it unsorted in `auto` mixed path
`setup()` sorts `dial.calibration_mapping` ascending by `.first` (raw). `angle_to_value()` assumes sorted pairs for the linear-scan branch. That is consistent currently — but **`angle_to_value()` is also called from the `debug_angle_calculation` + per-algorithm debug block inside `find_needle_angle()` for every algorithm**, and the wrap-around branch (`min_angle > max_angle`) returns *before* reaching calibration. A dial configured with BOTH `min_angle > max_angle` **and** `calibration_mapping` silently ignores calibration (early return). Wire-in is inconsistent.
- **Fix:** document/assert the precedence — calibration mapping must override range wrap-around, or reject the combination in the schema (`cv.Invalid`).

### B3 — Per-event `std::make_unique<ImageProcessor>` heap allocation inside `process_image_from_buffer()` hot path
`process_image_from_buffer()` allocates a fresh `ImageProcessor` (and internally buffers) **once per frame** (single `std::make_unique` call inside the dial loop — confirmed 1 site). `setup()` pre-allocates `persistent_buffer_`/`scratch_buffer_*` but **not** the `ImageProcessor`/results vector. §3.1/§5.3 mandate *no heap allocation during `loop()`* — allocate in `setup()`. This runs inside `loop()` → 🚨 BLOCKER.
- **Fix:** create a single reusable `ImageProcessor` (sized to the max crop across dials) in `setup()` and reuse it. If per-dial format differs, cache/pre-allocate per-dial instances, or use a `StaticVector`/fixed pool.

### B4 — JPEG fallback path (`decode_jpeg`) re-allocates when persistent buffer path fails
`scan` decodes into `persistent_buffer_`; if that fails it falls back to `ImageProcessor::decode_jpeg()` which does the same work plus a `memcpy`, defeating the zero-extra-alloc optimisation and still doing heap alloc in `loop()`. The fallback also triggers the *dynamic* path — acceptable, but it is exactly the fragmentation `setup()` tried to avoid.
- **Fix:** prefer a persistent double-buffer; only keep dynamic fallback as a one-time `setup()` decision, not a per-frame fallback.

### B5 — `scratch_buffer_` reused across dual uses with no invariant
`scratch_buffer_` is `reserve`d in `setup()` to `max_crop_area`; but during `process_image_from_buffer` it is `resize`d **per dial** and then `data()` handed to `apply_auto_contrast`/`apply_contrast` **and** the algorithm. `preprocess_image()` (multi_algorithm.cpp) also `safe_resize`s `this->scratch_buffer_` to full `w*h` (image size, not crop). Both `process_image_from_buffer` and `preprocess_image` share the same buffer with different size assumptions → aliasing/overwrite bug when `auto`/debug runs multiple algorithms.
- **Fix:** give preproc its own scratch vector (or a dedicated `preproc_scratch_`), and make per-dial scratch its own buffer with a documented size invariant.

### B6 — `detect_legacy` uses `FLT_MAX` sentinel + `kScanEndRadius` while others use `dial.max_scan_radius`
`detect_legacy()` hardcodes `end_r = radius * kScanEndRadius` (0.9) and `start_r = radius * 0.15f`, ignoring `dial.min_scan_radius`/`dial.max_scan_radius`/`deadzone_diameter`. `angle_to_value()` can stretch results outside the configured dial range. The 3 new algorithms read `dial.min/max_scan_radius` — so behaviour differs across algorithms in `auto` mode (inconsistent scan window).
- **Fix:** make `detect_legacy` honour `dial.min_scan_radius`/`max_scan_radius`/`deadzone_diameter` like the others.

### B7 — `set_camera_image_format(width,height,format)` decodes `substitutions["camera_resolution"]` with `map(int, rest.split("x"))` unvalidated
`__init__.py` `to_code()` parses a substitution string to ints with no try/except (unlike ssocr which wraps in try). A malformed substitution crashes codegen. The schema also does not bounds-check `crop_x/crop_w` etc. against the camera resolution.
- **Fix:** wrap parsing in try/except and add `cv.int_range` bounds / cross-field validation (`crop_x + crop_w <= img_width`).

### B8 — `value_sensor_` publishes on every frame without validator gate; no validation history
`process_image_from_buffer()` always `publish_state(total_value)` and sets `validation_coord_.set_dial_fraction()`. The `value_validator_coordinator` only exposes `validate_reading(int,...)`/`set_dial_fraction()` — never actually *called* to gate the publish (the value is published unconditionally). §3.5 (Sensor/Output Consumer) requires configurable `max_absolute_diff`/`max_rate_change`, bounded history, and exposing confidence/validation state.
- **Fix:** route `total_value` through `validation_coord_.validate_reading()` before publishing, gate on `valid`, and expose confidence/state entities when debug/validator present.

### B9 — Confidence is not exposed as a sensor on the component level
Only per-dial `confidence_sensor` exists (Alpha). §3.5/§12 require confidence/validation-state exposure. Global `value_sensor_` has no confidence companion.
- **Fix:** add optional `confidence_sensor` (and/or `state`) at component level, wired through `__init__.py`.

### B10 — `reads_` / `scratch_buffer_2_` pre-allocation mismatch
`setup()` reserves `scratch_buffer_2_` to `max_crop_area`; `multi_algorithm.cpp` `median_filter_3x3` uses `scratch_buffer_2_` sized to full `w*h`. Same aliasing family as B5 — `scratch_buffer_2_` is full-frame in preproc but crop-sized in `process_image_from_buffer`. Resize is per-call → heap alloc in `loop()` happens when full-frame > reserved crop.
- **Fix:** reserve both scratch buffers to `max(img_width*img_height, max_crop_area)` once in `setup()`, and stop resizing in loop.

---

## 3. Enhancements / new features

### E1 — Reuse one `ImageProcessor` across dials (memory + speed)
Beyond B3, cache the processor + its output buffer per unique `(crop_w, crop_h, channels)` configuration so repeated identical dials share work, reducing per-frame allocation to zero.

### E2 — Automatic algorithm selection tuning
With B1 fixed, add a config `algorithm: "auto"` that picks the best-confidence branch — but *normalise* scores so `legacy` (raw) isn't compared to top-hat (preprocessed) on different scales. Consider exposing per-algorithm confidence in debug logs and a recommended-per-dial override.

### E3 — Grayscale direct-stream path (§ todo.md / §3.3)
`todo.md` lists a 30% win from grayscale direct input (skip JPEG decode). `analog_reader` already has a `PIXFORMAT_GRAYSCALE` persistent-buffer path — extend the `current_format == "GRAYSCALE"` check to bypass `decode_jpeg` entirely when the camera can deliver raw grayscale (`camera_pixel_format` substitution), matching the new ESPHome raw-stream support.

### E4 — Stacked-digit robustness
The stacked-mode precision/rounding is already carefully guarded (scale validation, finite checks). Enhancement: add schema-level validation that stacked mode is only allowed when dials share a consistent scale ordering, and document the neighbour-carry semantics in `README.md`.

### E5 — Dial-aware validator wiring
`set_dial_fraction` is already fed to the validator. Enhancement: wire a `validate_reading(total_value, confidence, bool)` overload that returns validity so `value_sensor_` publish can be gated (ties to B8), plus `enable_dial_correction` config already present in `value_validator`.

---

## 4. Test / repo compliance gaps (all components in one sweep)

| Item | Status | Action |
|------|--------|--------|
| `cg.add_define("USE_ANALOG_READER")` | present | mirror in `esphome/core/defines.h` (repo can't; verify ESPHome core) |
| Build tests both ESP32 + S3 | present | ✅ but file names are `esp32.yaml/esp32-s3.yaml` — §9.1 wants `test.<platform>.yaml`; rename to `test.esp32-idf.yaml`/`test.esp32-s3-idf.yaml` across all 3 |
| CRLF / trailing whitespace | all files | repo-wide LF + whitespace sweep (§7.7 BLOCKER) |
| Per-frame heap alloc in `loop()` | B3/B5/B10 | fix above |

---

## 5. Priority ordering (bug-fix first)

1. B1 (UB correctness) — initialise `DetectionResult`, gate selection on `has_*`.
2. B3 (per-frame heap alloc — §3.1 🔴) — reusable `ImageProcessor` in `setup()`.
3. B5/B10 (scratch aliasing + full-frame sizing) — dedicated buffers, single pre-allocation.
4. B6 (legacy ignores scan-radius config).
5. B7 (codegen parse guard + schema bounds).
6. B8/B9 (validator gating + confidence/state sensors).
7. B2 (calibration vs wrap-around precedence).
8. E1–E5 enhancements after fixes land.
