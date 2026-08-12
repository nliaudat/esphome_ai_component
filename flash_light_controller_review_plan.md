# Flash Light Controller — Review Plan (Bug Fixes + Enhancements)

**Scope:** `C:\dev\esphome_ai_component\components\flash_light_controller`
**Reviewed against:** `C:\dev\esphome_ai_component\.ai\instructions.md` (authoritative, single source of truth)
**Component status per §2.2:** **Active** — new features allowed, tests/docs/performance required.
**Role rules that apply (§3):** universal rules (§3.1) + Sensor/Output Consumer (§3.5) via its `_controls.yaml` + the consumers it feeds (`meter_reader_tflite`, `analog_reader`, `ssocr_reader` via `flashlight_coordinator.*`).

---

## 1. Context captured (files read)

| File | Role |
|------|------|
| `flash_light_controller.cpp/.h` | The component under review |
| `__init__.py` | Config schema + codegen (adds `USE_FLASH_LIGHT_CONTROLLER`) |
| `flash_light_controller_controls.yaml` | Runtime HA entity example |
| `README.md`, `manifest.json` | Docs / metadata |
| `meter_reader_tflite/flashlight_coordinator.{h,cpp}` | Primary consumer — calls `initiate_capture_sequence`, `enable/disable_flash`, `is_active()` |
| `meter_reader_tflite/meter_reader_tflite.cpp` | Consumer call sites (`set_flash_pre_time`/`post_time`, `force_inference`, `capture_preview_sequence`) |
| `analog_reader` + `ssocr_reader` `flashlight_coordinator.*` | Cloned consumers (same pattern) |
| `tests/components/flash_light_controller/{common,esp32,esp32-s3}.yaml` | Existing build tests |

**Repo state note:** the entire repo is CRLF + heavy trailing whitespace (§7.7/§12.6 = BLOCKER, but repo-wide, not component-local). flash_light_controller matches the repo; a single repo-wide LF/whitespace sweep fixes all. README.md also has non-ASCII chars.

---

## 2. Bugs found in `flash_light_controller` itself

### B1 — `debug_` field is dead (wired but never read) — YAGNI / dead code
- `set_debug()` + the `debug:` YAML option store `debug_`, but **no code path reads it** — the flag does nothing.
- **Fix:** either wire it to real log output (see E1) or delete the field + the `debug` schema option + the `set_debug()` setter (`__init__.py` lines using it). Per §12.13 (YAGNI) prefer deletion unless debug logging is genuinely wanted.

### B2 — `schedule_timeout` template declared but never defined/used
- `flash_light_controller.h` declares `template<typename F> void schedule_timeout(uint32_t ms, F &&f);` — no definition in .cpp, no call site anywhere.
- **Fix:** remove the declaration (dead code, §12.14). The live code uses `Component::set_timeout` directly.

### B3 — TOCTOU / stuck `is_active_` with no recovery (§8.5)
- Consumer `update_scheduling()` does check-then-act: `if (!controller_->is_active()) { controller_->initiate_capture_sequence(...) }`. `initiate_capture_sequence` re-checks `is_active_`. On the ESPHome main-loop task this is benign, but:
  - `is_active_` is set `true` at the **start** of the sequence (before `enable_flash()`); if a timeout is lost (component teardown/reset, `esp_restart()`, or a dropped `set_timeout`) the flag can pin `true` forever → flash never re-triggers.
  - There is **no cancel/abort** API to recover a stuck or in-flight sequence.
- **Fix:** add `reset()`/`cancel_active_sequence()` (see E3) and/or an `is_active_` auto-expiry. Also log if `enable_flash()`/`disable_flash()` `perform()` silently fails.

### B4 — No 0-time guard for `set_timeout(0, ...)`
- Schema defaults (`cv.positive_time_period_milliseconds`) enforce >0 at **config** time, but the runtime `_controls.yaml` `set_action` lambdas allow `min_value: 0`. A runtime-set 0 makes `flash_pre_time_`/`flash_post_time_` 0, collapsing the sequence (light on → immediate callback → immediate off). Not a crash, but wrong behavior.
- **Fix:** clamp in setters (`max(pre,1)`) or guard in `initiate_capture_sequence`.

### B5 — `#ifdef USE_FLASH_LIGHT_CONTROLLER` mirror / silent-empty risk
- `__init__.py` calls `cg.add_define("USE_FLASH_LIGHT_CONTROLLER")` **unconditionally** when the component is loaded, and the `.h` is fully guarded by that define. The `.cpp` is unguarded but includes the guarded header.
- Per §3.1/§12.6 this define MUST be mirrored in `esphome/core/defines.h` for static analysis. If a build does not see the define, the whole header/class silently compiles to an empty stub — hard to debug.
- **Fix/verify:** confirm the define is registered in the ESPHome `esphome/core/defines.h` used by this project (not present locally in our checkout) and add a static_assert-friendly guard or a compile error if the class is missing. Keep the unconditional define (component activation is an approved unconditional category §7.4.1).

### B6 — Whitespace/encoding blockers (§7.7, §10, §12.6)
- All 6 files: CRLF line endings, trailing whitespace on ~90% of lines, `README.md` has non-ASCII chars.
- **Fix:** `pre-commit run --all-files` / `script/ci-custom.py` sweep to LF, strip trailing WS, ASCII-only, single EOF newline. Do this **repo-wide** (every file is CRLF) not just here, to avoid mixed endings.

---

## 3. Bugs in consumers caused/aggravated by this API (include in plan)

### B7 — `meter_reader_tflite` `set_flash_pre_time`/`set_flash_post_time` reset the other value
- `meter_reader_tflite.cpp:1061-1062`:
  - `set_flash_pre_time(ms)` → `set_timing(ms, 2000)` — clobbers post-time
  - `set_flash_post_time(ms)` → `set_timing(5000, ms)` — clobbers pre-time
- The `flash_light_controller_controls.yaml` `set_action` lambdas call the controller's `set_flash_pre_time`/`set_flash_post_time` directly (the controller stores both independently — fine), but the meter_reader path funnels through the coordinator and clobbers the paired value.
- **Fix (consumer, but flow fix):** either give the flash controller a single atomic `set_timing(pre, post)`, or fix the consumer to preserve the other param. Recommend touching the consumer to read-then-write, or better: have the coordinator call the controller's independent setters.

### B8 — Timing-semantics mismatch documented vs. executed (capture runs, then light stays on `post_time`)
- Header/README say "3. Call callback (capture) 4. Wait post_time (ensure light during capture)". In `initiate_capture_sequence`, post_time starts **after** the callback returns. If the capture callback is fire-and-forget (async — the coordinator's `force_inference`/`capture_preview_sequence` schedule frames asynchronously), the light turns off `post_time` after callback-return, **potentially mid-capture**.
- `capture_preview_sequence` compensates with `warmup >= 1000ms`; `initiate_capture_sequence` does not.
- **Fix:** document that the callback must block until capture completes, or add a small safety keep-alive / require the consumer to hold the flash during capture.

---

## 4. Enhancement plan

### E1 — Wire `debug` to real logs (resolves B1)
Guard `ESP_LOGD` in `initiate_capture_sequence`, `enable_flash`, `disable_flash`, and setter changes behind `this->debug_`. Guard with the YAML `debug:` option per §7.4.1 (define `DEBUG_FLASH_LIGHT_CONTROLLER` only when `debug: true` in `__init__.py`).

### E2 — Document runtime reconfiguration semantics
`set_flash_pre_time`/`set_flash_post_time` mutate the fields, but **already-scheduled timeouts captured old values** — live changes don't affect an in-flight sequence. Document "changes apply to the next sequence" (YAGNI: don't add live-restart unless required).

### E3 — Add `cancel_active_sequence()`/`reset()` (resolves B3)
Force-abort an in-progress/stuck sequence: cancel pending timeouts and reset `is_active_ = false`, so a new `initiate_capture_sequence` can start. Small, justified — the watch/panic usage needs it.

### E4 — Expose a state entity (§3.5 Sensor/Output Consumer)
Publish a `binary_sensor` (or `text_sensor`) `flash_active` so Home Assistant shows flash sequence state (`active`/`idle`/`ready`). This is the component's natural §3.5 role via its `_controls.yaml`. Grats: the `_controls.yaml` already has number entities for pre/post — add the state sensor in the same file or have `__init__.py` emit it.

### E5 — Fix `_controls.yaml` id/initial-value mismatches + document inclusion
- It references `${id_prefix}_flash_controller` (id must match the user's controller id) and `initial_value: 7000` for pre-time vs schema default `5000` — first boot state differs from configured default.
- It is a **standalone include** (not auto-loaded by `__init__.py`) and needs `id_prefix` substitution. Document in README how to include it and keep ids/prefixes in sync.

### E6 — Schema/codegen polish (§6, §7.4, §12.8)
- Add `MULTI_CONF` if multi-instance support is ever wanted (currently single — note it's intentional).
- `DEBUG_FLASH_LIGHT_CONTROLLER` macro gating (E1) must be added via `cg.add_define` in `to_code()` and mirrored in `core/defines.h`.
- All setters already `this->`-prefixed (good). No C-style casts, no `#define` constants, no `new/delete`, no `std::regex` — the component is clean on the style blockers except whitespace.

### E7 — Guard/no-light path logging
`initiate_capture_sequence` with a null `flash_light_` runs the callback immediately with only a `ESP_LOGW`. Add a debug-guarded log so users understand why the flash is skipped (E1 wiring covers the path).

---

## 5. Testing (per §9)
- Existing tests exist for both ESP32 and ESP32-S3 (good). Add a `common.yaml` variant **without** `flash_light` (null-light fallback path) and one **with** `debug: true` (E1 macro) to exercise the new/guarded code.
- After any change: `./script/test_build_components.py -c flash_light_controller -t esp32` and `-t esp32-s3`.

## 6. Suggested fix ordering
1. **B6 whitespace/encoding** (repo-wide sweep) — commits are blocked until clean per §11 BLOCKER.
2. **B1/E1** wire-or-delete `debug` (pick one).
3. **B2** remove dead `schedule_timeout`.
4. **B3/E3** cancel/recovery API.
5. **B7** fix consumer `set_timing` clobber (cross-component).
6. **B4** zero-time guard.
7. **E4/E5** state entity + controls docs.
8. **B5** verify `USE_FLASH_LIGHT_CONTROLLER` mirrored in `defines.h`.

## 7. Review-priority mapping (§11)
- 🔴 **BLOCKER:** B5 (silent empty compile risk), B6 (whitespace/encoding), B1/B2 dead code if left.
- 🟠 **WARNING:** B3 (stuck state, recoverability), B4 (0-time), B7 (consumer clobber — correctness), B8 (capture timing semantics).
- 🟡 **INFO:** E2/E4/E5/E6, naming, docs.

## 8. Communication note (§12.11)
No changes committed without explicit approval (§12.10). Present as proposals; confirm `debug` wiring choice and consumer `set_timing` fix before implementing.

---

## 9. Verification & Resolution Status (2026-08-12)

**Repo paths corrected:** scope is `components/flash_light_controller` under `c:\Users\nl\Dropbox\home_automation\esphome_ai_component` (not `C:\dev\...`).

### Findings corrected against the actual code

| Item | Status | Note |
|------|--------|------|
| B1 | ✅ FIXED | `debug_` now gates `ESP_LOGD` in `.cpp` (no-debug path also has a `ESP_LOGW`); choice: wire, per plan §2 B1 option "real log output". |
| B2 | ✅ FIXED | Template was in the **consumers'** `flashlight_coordinator.h` (meter_reader_tflite, analog_reader, ssocr_reader), NOT `flash_light_controller.h`. Removed from all 3; `grep schedule_timeout` = NONE. |
| B3 | ✅ FIXED | Added `FlashLightController::cancel_active_sequence()` using named timeouts `"flash_capture"`/`"flash_off"` + `Component::cancel_timeout(name)`, turns flash off, resets `is_active_`. |
| B4 | ✅ FIXED | `set_flash_pre_time`/`set_flash_post_time` clamp `0 → 1`; `_controls.yaml` `min_value` changed `0 → 1`. |
| B5 | ⚠️ VERIFIED RISK | `USE_FLASH_LIGHT_CONTROLLER` NOT in installed ESPHome `esphome/core/defines.h` (`C:\Users\nl\AppData\Local\Programs\Python\Python312\Lib\site-packages\esphome\core\defines.h` grep = 0). Runtime is safe because `__init__.py:36` adds the define unconditionally. **Static-analysis mirroring gap remains** — requires upstream ESPHome change, out of scope for this repo. |
| B6 | ⚠️ STALE | Byte-check: all flash_light_controller source files are **LF**, not CRLF. No whitespace blocker currently. (Other repo files may still be CRLF; not re-verified.) |
| B7 | ✅ FIXED | `meter_reader_tflite.cpp` `set_flash_pre_time`→`set_timing(ms, get_post_time())`, `set_flash_post_time`→`set_timing(get_pre_time(), ms)`; added `FlashlightCoordinator::get_post_time()` (real + dummy branches). |
| B8 | 📄 DOCUMENTED | Header sequence doc now states the callback must block until capture completes; async consumers must hold the flash themselves. |
| E1 | ✅ DONE | `debug` flag wired (see B1). |
| E2 | ✅ DONE | Header documents runtime changes apply to the **next** sequence. |
| E3 | ✅ DONE | see B3. |
| E4 | ⏳ DEFERRED | `flash_active` state entity not added; requires `binary_sensor` schema/codegen in `__init__.py` — larger scope. Left as INFO per §7. |
| E5 | ✅ PARTIAL | `initial_value` `7000 → 5000` (matches schema `5s`), `min_value` `0 → 1`. Id-prefix/docs note remains (standalone include). |
| E7 | ✅ DONE | covered by B1 debug-gated logging + existing `ESP_LOGW` fallback. |

### Review-priority mapping (updated)
- ✅ Fixed now: B1, B2, B3, B4, B7 (and E1/E2/E3/E5/E7).
- ⚠️ Remaining open: B5 (upstream `defines.h` mirror), B8 (consumer-side contract — documented), E4 (state entity — deferred INFO), E6 (MULTI_CONF — intentional single).
- 🔴 BLOCKER status re-checked: no current whitespace/encoding blocker (B6 stale); B5 dotnet-risk documented.

### Tests (§5) — corrected
- Existing build tests found: `tests/components/flash_light_controller/{common,esp32,esp32-s3}.yaml` (variant naming differs from `test.esp32-idf.yaml` used by other components; `script/test_build_components.py` referenced in the original plan **does not exist** in this checkout — actual scripts are `ci-custom.py`, `run_lint.py`, `helpers.py`).
- Recommended: run the component's `esp32.yaml` / `esp32-s3.yaml` through the repo's `esphome compile` flow (or `script/ci-custom.py`) to exercise the new `cancel_active_sequence`, debug-gated logging, and clamped setters.
- `python -m py_compile components/flash_light_controller/__init__.py` ✅; controls YAML parses ✅.
