# SSOCR Reader — Review Plan (Bug Fixes + Enhancements)

**Scope:** `C:\dev\esphome_ai_component\components\ssocr_reader`
**Reviewed against:** `C:\dev\esphome_ai_component\.ai\instructions.md` (authoritative)
**Component status per §2.2:** **Alpha** — API may change, must stay opt-in & documented.
**Role rules that apply (§3):** universal (§3.1) + Camera Consumer (§3.3) + Sensor/Output Consumer (§3.5) + Network indirect (none) — ssocr consumes camera, publishes sensors, opts into `value_validator`.

---

## 1. Context captured (files read)

| File | Role |
|------|------|
| `__init__.py` | Config schema + codegen (`USE_SSOCR_READER`) |
| `ssocr_reader.{h,cpp}` | Polling/camera-listener + 7-seg recogniser |
| `camera_coordinator.{h,cpp}` | Window control + ImageProcessor wrapper |
| `flashlight_coordinator.{h,cpp}` | Flash scheduling wrapper |
| `value_validator_coordinator.{h,cpp}` | `value_validator` opt-in wrapper |
| `README.md`, `manifest.json` | Docs / metadata |
| `tests/components/ssocr_reader/{common,esp32,esp32-s3}.yaml` | Build tests |

---

## 2. Bugs found in `ssocr_reader`

### B1 — `confidence_sensor_` setter exists but **no schema option / no wiring**
`ssocr_reader.h` declares `set_confidence_sensor()` and `ssocr_reader.cpp` publishes `confidence_sensor_->publish_state(100.0f)` on a valid read — but `__init__.py` never defines a `confidence` YAML option, so the sensor can't be configured → dead feature, and §3.5 confidence exposure is unmet.
- **Fix:** add `optional confidence: sensor.sensor_schema()` + codegen wiring in `__init__.py`, or drop the feature. §3.5 wants confidence exposed, so **add** it.

### B2 — `setup()` does not validate crop bounds before `update_image_processor_config`
`effective_crop_w = crop_w_>0 ? crop_w_ : img_width_ - crop_x_`; `effective_crop_h = crop_h_>0 ? crop_h_ : img_height_ - crop_y_`. If the user sets an out-of-range `crop_x`/`crop_y` (defaults 0,0 → full crop) there is no bounds check; `process_image` computes `cw = w - cx`, `ch = h - cy` and pushes `{cx, cy, cx+cw, cy+ch}` into zones. `analysis/crop` may read past the ROI width/height.
- **Fix:** schema `cv.int_range` bounds (crop_x ≤ img_width, crop_y ≤ img_height, crop_w/crop_h ≤ remainder) + a `setup()` clamp/`mark_failed()` guard mirroring analog_reader §8.1 pattern.

### B3 — No guarded-multiplication overflow check on `roi_w * roi_h`
`process_image()` does `static_cast<size_t>(roi_w) * static_cast<size_t>(roi_h)` for the binarise loop and confidently indexes `raw_roi[i*roi_w + j]`. On 32-bit ESP32 the product is size_t (32-bit) — an oversized crop could overflow the multiplication (§8.1 CVE-2026-23833 pattern required).
- **Fix:** apply the guarded-multiplication pattern from §8.1 before the binarise/vertical-projection loops.

### B4 — `recognize_digit` per-call `segments[7]` and `digit_map[10]` static tables are fine, but the 3×3 probe uses `count` from `(nx,ny)` in-bounds only
The mask probe: samples a 3×3 window around each segment point, dividing `on_pixels`/`count`. Near the ROI edge `count < 9`, normalising the ratio up — a pixel at the border counts fewer samples → biased ratio. Also `img[ny * stride + nx]` uses `stride` = `roi_w` for the *column* stride but `recognize_digit` is called with `img = raw_roi + d_x` and `w = d_w` while assuming `stride` covers the full row ROI. This is correct (stride=roi_w) — just note border bias.
- **Fix:** pad/clamp the 3×3 window (sample only valid neighbors but keep denominator = sampled count, which it does), and use a fixed 3×3 with explicit boundary handling so ratios stay comparable.

### B5 — Digit segmentation `has_signal = col_sums_[j] > (roi_h * 0.05)` — magic threshold, no config
`roi_h * 0.05` (5% of height) hard-coded, plus `j - start_x > 5` and `d_w < 3` all magic numbers. `product_of` decimal point (`CONF_DECIMAL_POINT` defined in `__init__.py` but **never used in code**) — the schema accepts `decimal_point` but `ssocr_reader.h/.cpp` doesn't expose it, and `process_image` never handles a decimal separator. Dead config option.
- **Fix:** expose `decimal_point` to code (or remove from schema — §12.13), and make segmentation thresholds configurable (`threshold_level` exists; add `min_digit_width`/`gap_tolerance`).

### B6 — `confidence_sensor_` is published as **hardcoded 100.0f** on valid, no per-digit confidence
Even if B1 wires the sensor, the recogniser does exact-mask match only — it returns `-1` (unknown) or a digit, never a per-digit probability. `confidence` is a binary 100 or fail. §3.5/§12 wants real confidence.
- **Fix:** compute a confidence from mask match quality (e.g. how close the probe ratio was to the match threshold) and expose it; gate publishing on a configurable minimum confidence.

### B7 — `result_str` has no decimal-point / negative support
`strtol(result_str.c_str(), ...)` yields `long` → cast to `int`. `digit_count` is bounded 1–16, so `int` is fine, but `I`/`O`/non-numeric handling and `?` rejection skip the publish (correct) yet `decimal_point` never affects parsing.
- **Fix:** once decimal_point lands, parse the string with a threshold-aware routine that validates the whole sequence (digit group + decimal) before calling `validate_reading`/publish.

### B8 — Watchdog/timeout in `loop()` mirrors analog_reader, but no frame-request → camera recovery
`loop()` has the atomic state `REQUESTED→…IDLE` watchdog (✅ TOCTOU-free). But unlike `meter_reader_tflite` §3.2 camera-recovery guidance, ssocr has no `esp_restart()`/re-init after repeated timeouts. `camera_coordinator::set_window/reset_window` block with `delay()` (documented exception, ✅), and `test_camera_after_reset`/`basic_recovery` exist but are **never called** from `loop()`.
- **Fix:** wire a consecutive-timeout counter in `loop()` that calls `camera_coord_.basic_recovery()`/`test_camera_after_reset()` after N≥3 consecutive timeouts, with `esp_restart()` as last resort — mirror §3.2.

### B9 — `CameraCoordinator::process_frame` returns `std::vector<ProcessResult>` freshly each call
`std::vector<ProcessResult>` built per-frame in `process_image()` (heap alloc in `loop()`; §3.1/§5.3 🔴).
- **Fix:** pre-size a member vector or return a reference/fixed buffer; allocate in `setup()`.

### B10 — `on_camera_image` + `update` TOCTOU checked (✅), but `paused_` support absent
Unlike analog_reader, ssocr has **no `paused`/`set_pause_processing`** and `on_camera_image` has no paused gate. Feature parity gap; `update()` also has no watchdog on the flashlight legacy path re-arm.
- **Fix (enhancement):** add `paused_` + `set_pause_processing(bool)` consistent with analog_reader.

---

## 3. Enhancements / new features

### E1 — Per-digit confidence (§3.5)
Once B6 is in, gate publishing on min-confidence and surface a `confidence` sensor with real probabilities (mask-distance-ratio), not hardcoded 100.

### E2 — Decimal-point & ± sign support
Wire `decimal_point` into parsing + segmentation (B5), and support a leading `-`/`+` or `?` fallback. Keep `?`→skip-publish policy.

### E3 — Rotation support
`camera_coordinator` has a `rotation_` field + `set_rotation()` setter that is **never called** from `ssocr_reader` (no schema option). Add rotation config, applying it in `update_image_processor_config` (config.rotation) and the preview path, mirroring §3.3 rotation behavior.

### E4 — Camera window config
`camera_coordinator::set_window_config/apply_window/supports_window` exist but `__init__.py`/`ssocr_reader` never wire them. Add optional `crop`+`window` config so OV2640/OV3660 windowing works incl. rotation — §3.3 windowing rule.

### E5 — Binarisation method selection
Schema already accepts `threshold_type: fixed|otsu` (defined `CONF_THRESHOLD_TYPE`); the code only implements fixed threshold. Implement an Otsu path (or document fixed-only) to honour the option; add config for auto-threshold if wanted.

---

## 4. Test / repo compliance gaps

| Item | Status | Action |
|------|--------|--------|
| `cg.add_define("USE_SSOCR_READER")` | present | mirror in defines.h |
| Build tests both ESP32/S3 | present | ✅ rename `esp32/esp32-s3.yaml` → `test.esp32-idf`/`test.esp32-s3-idf` (§9.1) — repo-wide |
| CRLF / whitespace | all files | repo-wide LF + whitespace sweep |
| `CONF_DECIMAL_POINT` defined but unused | dead option | wire or remove (§12.13) |
| `CONF_THRESHOLD_TYPE` defined but unimplemented | mismatch | E5 |

---

## 5. Priority ordering (bug-fix first)

1. B9 (per-frame heap alloc — §3.1 🔴) — reusable result buffer in `setup()`.
2. B3 (overflow guard on ROI size — §8.1 🔴).
3. B2 (crop-bounds validation in schema + setup).
4. B6/B7 (real confidence + decimal-point wiring).
5. B8 (consecutive-timeout camera recovery).
6. B4 (border-ratio bias in recogniser).
7. B5 (segmentation magic numbers into config; wire/remove `decimal_point`).
8. B1 (confidence sensor wiring in schema).
9. E1–E5 enhancements after fixes land.
