# tflite_micro_helper — Review & Enhancement Plan

> Review conducted against `.ai/instructions.md` (SINGLE SOURCE OF TRUTH),
> `.ai/instructions.yaml`, `.greptile/rules.md`, and
> `tflite_micro_helper_update_plan.md`.
> Note: `.ai_agent_requirements.md` (referenced in the task) does not exist on disk;
> the actual agent instructions live in `.ai/instructions.md` + `.ai/instructions.yaml`.

Status: **Active** (§2). Version 2.0.0 (`Seen in manifest.json`).

---

## 1. Summary

`tflite_micro_helper` is already far along the `update_plan.md` migration: it is a
self-contained multi-source loader (local / `github://` / http manifest), supports
`model_type: image|audio`, CRC32 verification, PSRAM-aware RAII arena, X-Macro op
registration, and is consumed by `meter_reader_tflite` via ID reference. The remaining
work is mostly **hardening, dead-code removal, and completing the audio path** — plus
fixing 3 real bugs and closing test-coverage gaps.

Files reviewed (all 15):
`tflite_micro_helper.{h,cpp}`, `model_handler.{h,cpp}`, `model_handler_debug.cpp`,
`memory_manager.{h,cpp}`, `op_resolver.{h,cpp}`, `tflm_operators.h`,
`debug_utils.{h,cpp}`, `__init__.py`, `README.md`, `manifest.json`.

---

## 2. Findings (priority per instructions.md §11)

### 🔴 BLOCKER

B1. **`probe_arena_size_()` is dead code AND stems from a buggy double-memory
    implementation.** Declared in `model_handler.h`, defined in `model_handler.cpp`
    (2-occurrence: decl + def), but **never called**. The live path still uses the
    manual `ESP32-S3 1.5x bump` in `TFLiteMicroHelper::load_model()` — exactly the
    mechanism `update_plan.md §C` says to replace with arena probing. The existing
    `probe_arena_size_` itself is also broken:
    - allocates with raw `MALLOC_CAP_8BIT` (not PSRAM-aware, ignores `MemoryManager`);
    - the binary-search shrink loop is incorrect (`upper = lower+16` then `break` even
      when `ok` is for `lower`, and the `while(lower<upper)` can return `upper` that
      was never proven to allocate);
    - takes a `resolver` it then never uses to construct the probe interpreter with the
      **5-arg/4-arg mismatch** (uses the 4-arg form even in the streaming build).
    **Action:** wire a corrected PSRAM-aware `probe_arena_size_()` into the load path
    (per plan §C) and delete the 1.5x-arena bump. See enhancement E1.

B2. **C-style cast in `debug_utils.cpp` (`log_arena_usage`).**
    `(float) used / total * 100.0f` violates §3.1/§7 blocker (no C-style casts).
    **Action:** `static_cast<float>(used)`. (E4)

### 🟠 WARNING

W1. **TOCTOU between `is_model_loaded()` and `load_model()` (CWE-367).**
    `TFLiteMicroHelper` uses `std::atomic<bool> model_loaded_` but the load sequence is
    not a single atomic state machine: `load_model()` does multi-step non-atomic work
    (CRC → arena alloc → `load_model_with_arena` → flag set). `meter_reader_tflite`
    calls `is_model_loaded()` then `load_model()` (two separate atomics / check-then-act)
    — the classic §3.1/§8 TOCTOU. Also `set_model`, `set_tensor_arena_size`,
    `set_expected_crc32` race with `load_model`. **Action:** E2 (introduce a
    load/unload lock, or a single `std::atomic<enum LoadState>`; prefer the latter to
    satisfy the "single atomic state machine" rule).

W2. **Helper state is not reset on `unload_model()`.** `unload_model()` resets
    `model_loaded_` and the stats cache, but **not** `model_data_`, `model_length_`,
    `expected_crc32_`, `tensor_arena_size_requested_`, `input_config_*`, or
    `arena_bumped_`. Contrast with `ModelHandler::unload()` which fully resets. This
    breaks clean re-load of a *different* model: the stale `expected_crc32_` (or stale
    arena size / `arena_bumped_`=true preventing the S3 bump) persists. **Action:** E3.

W3. **`get_model_spec()` mis-handles audio/3D tensors and guesses `input_type`.**
    `get_input_width/height/channels()` return `0` when `dims->size < 4` (audio = 3D),
    so `num_elements = 0*0*0 = 0` and `input->bytes == 0*4` can mis-classify the model
    as uint8. The `input_type` is inferred from `input->bytes == num_elements*4`
    (a hardcoded 4-byte heuristic) instead of `input->type` directly. **Action:** E5:
    classify from `input->type`; guard audio shapes and document `ModelSpec` as
    image-centric (audio consumers should read `input_tensor()->dims` themselves).

W4. **Audio mode is a configuration-only stub in the C++ wrapper.**
    `TFLiteMicroHelper` implements the audio *setters* (populating `audio_config_`)
    but has **no runtime** `perform_streaming_inference()` / `determine_detected()` /
    `reset_probabilities()` / `get_unprocessed_probability_status()` that
    `update_plan.md` listed under `#ifdef USE_TFLITE_STREAMING`. `USE_TFLITE_STREAMING`
    is defined by `__init__.py` for `audio`, and `audio_config_` is written but never
    read anywhere. This is consistent with the plan's "consumer owns MRV/streaming"
    decision, but the helper advertises audio support without any testable runtime
    surface. **Action:** E6 (decide: expose streaming runtime in the helper, or
    explicitly document the helper as *load/configure-only* and move audio runtime into
    the consumer — required before `micro_wake_word` can consume it).

W5. **`MAX_OPERATORS` build flag is global, not per-model (MULTI_CONF race).**
    `to_code()` calls `cg.add_build_flag(f"-DMAX_OPERATORS={model_ops}")` once per
    component instance. With `MULTI_CONF=True` and multiple models of different op
    counts, **the last `add_build_flag` wins** — earlier models silently get a
    too-small/too-large resolver. `MAX_OPERATORS` is a compile-time macro, so this is a
    genuine correctness bug for the documented multi-model use case. **Action:** E7 —
    pin a single safe `MAX_OPERATORS` per build (max across configured models, +5
    headroom), since the macro is process-wide.

W6. **`process_output(const float*)` dereferences `output_data[0]` with no null guard.**
    The `TfLiteTensor*` overload guards null, but the public `const float*` overload
    reads `output_data[0]` (and loops to `output_size_`) with no null check. A direct
    call with `nullptr` segfaults. **Action:** E8.

### 🟡 INFO

I1. **Dead code:**
    - `ModelHandler::probe_arena_size_` (until E1 wires it) — unreferenced.
    - `MemoryManager::parse_size_string` — Python `datasize_to_bytes` already converts;
      the C++ string-parser is never called.
    - `ModelHandler::validate_model_config()` — defined, never invoked.
    - `debug_utils::log_model_info` / `log_inference_time` / `log_arena_usage` are
      effectively unused outside debug builds (verify before delete).
    **Action:** delete or wire (E9).

I2. **`infer_model_config_from_filename()` relies on unenforced filename conventions**
    (`_GRAY/_RGB/_BGR/_10cls/_100cls`). Fragile but documented; keep as last-resort
    fallback only. INFO.

I3. **`set_probability_cutoff` truncation:** `static_cast<uint8_t>(f * 255.0f)` with no
    clamp/NaN guard. `cv.percentage` clamps [0,1] so YAML is safe, but a C++ consumer
    passing >1.0/NaN truncates silently. Add a saturating clamp. (E10)

I4. **`tflm_operators.h` marks several common ops unavailable** (`POW`, `SELECT`,
    `WHERE`, `REDUCE_MIN/PROD/ALL/ANY`, `REVERSE_SEQUENCE`, `WHERE`, `BIDIRECTIONAL_*`)
    — this limits which audio/non-image model graphs can load. If `esp-micro-speech-features`
    / wake-word models need any of these, update the X-Macro list against
    `esp-tflite-micro` 1.3.7 (per the file's own "cross-reference" note).

I5. **`get_input_*` helpers are image-centric** (`dims->size < 4` → 0). Audio consumers
    must use `input_tensor()->dims` directly. Document this contract. (E11)

I6. **Tests are audio-only and remote-only.** `tests/components/tflite_micro_helper`
    has `test.esp32-idf.yaml` + `test.esp32-s3-idf.yaml` (good §9.1 dual-target), but
    `common.yaml` only exercises **one audio model** via HTTP — no image model, no local
    file, no `github://`, no `debug:true`, no multi-model, no `TFLITE_FORCE_SRAM` /
    `TFLITE_FORCE_PSRAM` override, no image YAML. (E12)

I7. **`DEPENDENCIES` / `esp32` guard:** `__init__.py` only emits code
    (`USE_TFLITE_MICRO_HELPER`, model PROGMEM, build flags, IDF deps) inside
    `if CORE.target_platform == "esp32"` and `DEPENDENCIES = ["esp32"] if ... else []`.
    This leaves ESP8266/other targets with a config that silently adds no C++ — likely
    intended (ESP8266 unsupported §4), but should emit a clear `cv.Invalid` or warning
    on non-esp32 rather than silently no-op. (E13)

---

## 3. Enhancement Plan (ordered)

Each item states **what / why / how**, and respects the repo's non-negotiables
(`this->`, `constexpr`, RAII, 16-byte aligned arena, dual-target, `Debug same guard`).

### E1 — Wire arena probing into the load path (replaces 1.5x bump) 🔴
**What:** make `MemoryManager`/`ModelHandler` provide a *correct* PSRAM-aware
`probe_arena_size()` and call it from `TFLiteMicroHelper::load_model()`; delete the
`CONFIG_ESP32S3_DATA_CACHE*` manual `*3/2` bump (`arena_bumped_`).
**Why:** plan §C explicitly replaces the S3 cache bump; the current probe is buggy/dead.
**How:**
- Rewrite `ModelHandler::probe_arena_size_()` to use `MemoryManager::allocate_tensor_arena()`
  for probing so PSRAM is honored, and drop the unused `resolver` parameter (or pass
  `&*resolver_` if required by the 5-arg streaming interpreter).
- Fix the shrink loop: probe with the **same** interpreter constructor variant as the
  real load (4-arg vs 5-arg MRV).
- Per plan §C: try manifest/aligned size → 1.5x → 2x, then binary-search down to
  `arena_used_bytes() + 16` (16-byte aligned). Return 0 on total failure.
- Remove `arena_bumped_`, the `#if defined(CONFIG_ESP32S3_DATA_CACHE_64KB) ...` block in
  `tflite_micro_helper.cpp`, and the `TFLITE_FORCE_*` trade-offs stay in `MemoryManager`.
- Keep `TFLITE_FORCE_SRAM`/`TFLITE_FORCE_PSRAM` override path (documented, unchanged).

### E2 — Single atomic load state machine (TOCTOU) 🟠
**What:** replace `std::atomic<bool> model_loaded_` + ad-hoc `arena_stats_mutex_` with a
single atomic state (`enum LoadState { UNLOADED, LOADING, READY }`) OR add a mutex that
spans the whole load/unload critical section.
**Why:** §3.1/§8 — no check-then-act across multiple atomic flags.
**How:** keep `is_model_loaded()`/`load_model()`/`unload_model()` lock-free by using one
atomic enum; make `load_model()` transition `LOADING→READY` only on success and
`LOADING→UNLOADED` on failure; document which methods may run from the consumer loop
vs `setup()` (§3.3 — no heap alloc in `loop()`; arena alloc is `setup()`-only here).

### E3 — Full state reset in `unload_model()` 🟠
**What:** reset `model_data_`, `model_length_`, `expected_crc32_`,
`tensor_arena_size_requested_` (to documented default or 0), `input_config_*`,
`audio_config_`, and `arena_bumped_` on unload — mirroring `ModelHandler::unload()`.
**Why:** clean reload of a different model; stale CRC/arena-size currently persist.
**How:** factor a private `reset_state_()` used by `unload_model()`.

### E4 — C-style cast fixes 🔴
**What:** `debug_utils::log_arena_usage`: `static_cast<float>(used)`.
**Why:** §3.1/§7 blocker.
**How:** mechanical; add to E1 branch to touch only debug_utils.cpp.

### E5 — `get_model_spec()` accuracy for float/audio 🟠
**What:** derive `input_type` from `input_tensor()->type` (`kTfLiteFloat32` → float,
else quantized), not from `bytes == num_elements*4`. Handle `dims->size < 4` (audio
3D) by returning the raw tensor dims (or flags) instead of `0` values.
**Why:** §3.2 — dimensions/type must come from `TfLiteTensor` at runtime; the heuristic
mis-reports audio/float models.
**How:** 
```cpp
spec.input_type = (input && input->type == kTfLiteFloat32) ? 1 : 0;
```
and clamp the `num_elements` path so 3D audio doesn't divide by zero.

### E6 — Decide/complete the audio runtime surface 🟠
**What:** either (a) add `perform_streaming_inference` /
`determine_detected` / `reset_probabilities` / `get_unprocessed_probability_status`
to `TFLiteMicroHelper` under `#ifdef USE_TFLITE_STREAMING`, or (b) explicitly document
the helper as *image single-shot + audio configuration-only*, and move streaming MRV
state into the consumer.
**Why:** `USE_TFLITE_STREAMING` is defined and `audio_config_` is set but unread; the
helper's audio story is incomplete, blocking `micro_wake_word` (§2 README already
advertises it).
**How:** recommended = (b) per `update_plan.md` ("No Separate StreamingModel Class",
consumer owns MRV + var_arena): delete the unused `audio_config_` application from the
helper (or keep setters as validation-only + pass-through), and add a documented
`#include`/example showing the consumer owns `MicroResourceVariables` + 5-arg
`load_model_with_arena(mrv)` path (already implemented in `model_handler.cpp`). Keep the
`USE_TFLITE_STREAMING` define for consumer components.

### E7 — Single `MAX_OPERATORS` for multi-model builds 🟠
**What:** in `__init__.py`, compute the max operator count across all `tflite_micro_helper`
entries (+5 headroom) and add **one** `-DMAX_OPERATORS=N` build flag.
**Why:** `add_build_flag` is process-global; per-instance adds race/overwrite (W5).
**How:** two-pass in `to_code` over `config.get(DOMAIN, [])` (collect counts) then emit
the single flag after processing all entries.

### E8 — Null-guard `process_output(const float*)` 🟠
**What:** add `if (!output_data) { ESP_LOGE(...); return {0,0}; }`.
**Why:** W6, §8.1 bounds/null safety.
**How:** trivial guard.

### E9 — Remove/retire dead helpers 🟡
**What:** delete `MemoryManager::parse_size_string`, `ModelHandler::validate_model_config`
(until E1 wires `probe_arena_size_`), and the unused `debug_utils*` log helpers after
verifying no callers via a repo-wide grep.
**Why:** §11 INFO (dead code) — keeps binary small and maintenance honest.

### E10 — Saturating `set_probability_cutoff` 🟡
**What:** clamp `f` to [0,1] (and NaN→default) before the uint8 cast.
```cpp
f = std::max(0.0f, std::min(1.0f, f));
```
**Why:** I3 — robust for C++ consumers even though YAML `cv.percentage` already clamps.

### E11 — Document the image-centric `ModelSpec`/`get_input_*` contract 🟡
**What:** add Doxygen/comment that `get_input_width/height/channels`,
`get_model_spec`, and `ArenaStats` are **image (4D) semantics**; audio consumers must
read `input_tensor()->dims` and manage their own MRV.
**Why:** I5 — prevents future audio consumers from misusing 0-returning dims.

### E12 — Expand tests to both model types + sources 🟡
**What:** add to `tests/components/tflite_micro_helper/common.yaml` (and/or new files):
- an **image** model entry (local `.tflite` + `.txt` in `tests/` or `models/`);
- a `github://` source entry;
- a `debug: true` entry (now `DEBUG_TFLITE_MICRO_HELPER` compiles `model_handler_debug.cpp`);
- exercise `TFLITE_FORCE_SRAM` / `TFLITE_FORCE_PSRAM` via `sdkconfig`/build args in at
  least one YAML;
- a **multi-model** image+audio case to catch the E7 `MAX_OPERATORS` race.
Existing `test.esp32-idf.yaml` / `test.esp32-s3-idf.yaml` already give the §9.1
dual-target compile coverage — keep them.
**Why:** §3.1/§9.1 — the image path (single-shot inference + grayscale) is currently
untested; the audio remote-only test never touches `TFLiteMicroHelper`'s C++ setter path
with local data.

### E13 — Non-esp32 path: explicit error 🟡
**What:** in `__init__.py`, raise `cv.Invalid` (or log a clear warning + skip) when
`CORE.target_platform != "esp32"`, since the component only generates code for esp32.
**Why:** I7 — currently silently no-ops on ESP8266/other targets.

---

## 4. Sequencing & review priorities

1. **E1 + E2 + E3 (bugs, BLOCKER/WARNING)** — wire probed arena, atomic state, state
   reset. These are the highest-value robustness fixes and match `update_plan.md §10`
   ordering (refactor load path first).
2. **E5, E8 (accuracy/safety)**, then **E7 (multi-model flag)** before enabling any new
   audio consumer.
3. **E4 (cast), E9 (dead code)**, then **E6 (audio runtime decision)** — do E6 before
   `micro_wake_word` integration so the contract is settled.
4. **E10–E13 (hardening/docs/tests)** — low risk, high coverage value. Land tests (E12)
   alongside E1 so the probed-arena change is compile/link covered on both ESP32 & S3.

## 5. Do NOT change
- `MemoryManager::allocate_tensor_arena`'s `heap_caps_aligned_alloc(16, ...)` and the
  PSRAM-only/no-fallback policy (§3.2 warning) — preserve the `TFLITE_FORCE_SRAM/PSRAM`
  debug overrides.
- The X-Macro `tflm_operators.h` registration scheme and `MAX_OPERATORS` default (30)
  when no `.txt` — keep `-DMAX_OPERATORS` derivation, but fix its multi-instance emission (E7).
- The `github://` and http-manifest source resolution + `external_files` caching in
  `__init__.py` (the plan's target state) — only add missing-source tests.
- The consumer contract used by `meter_reader_tflite` (`set_tflite`, `load_model`,
  `run_inference`, `get_model_spec`, `get_arena_stats`, `debug_test_parameters`) —
  any E5/E6 change must keep these signatures backward-compatible.
