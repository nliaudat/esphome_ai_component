# Data Collector — Review Plan (Bug Fixes + Enhancements)

**Scope:** `C:\dev\esphome_ai_component\components\data_collector`
**Reviewed against:** `C:\dev\esphome_ai_component\.ai\instructions.md` (authoritative)
**Component status per §2.2:** **Active** — full support, new features allowed.
**Role rules that apply (§3):** universal (§3.1) + Network Consumer (§3.4) + Camera Consumer (§3.3 — indirectly reads the camera buffer via `collect_image`), plus `data_extractor` Python scripts (ruff/flake8).

---

## 1. Context captured (files read)

| File | Role |
|------|------|
| `__init__.py` | Config schema + codegen (`USE_DATA_COLLECTOR`, esp_http_client idf component) |
| `data_collector.{h,cpp}` | Queue + FreeRTOS uploader |
| `README.md`, `manifest.json` | Docs / metadata |
| `data_extractor/*.py` | Active-learning Python pipeline (extractor/dedupe/correct/clean/low_confidence) |
| `server/` | Local upload server helper |
| `tests/components/data_collector/{common,esp32,esp32-s3}.yaml` | Build tests |

---

## 2. Bugs found in `data_collector`

### B1 — Rate-limit timestamp is set on *enqueue*, not on *successful upload*
`collect_image()` checks `last_upload_time_` (minus `MIN_UPLOAD_INTERVAL_MS`) early; `upload_image()` sets `last_upload_time_ = now` when the job is **queued**, before `process_upload_sync()` actually performs the HTTP POST. If the upload fails (HTTP error, network drop) the same 60s window is consumed anyway → silent drops. §3.4 requires "log upload errors" and non-blocking; queueing is fine, but the window should reflect *committed/failed* status.
- **Fix:** keep the 60s window tied to a **committed** upload flag set after `process_upload_sync()` returns `true`, and add a separate short "queue retry" allowance so failures are retried, not rate-limited away.

### B2 — `collect_image()` early returns **before** updating `last_upload_time_`
Same family: the rate-limit check runs in `collect_image()`, but `upload_image()` (which sets the timestamp) is reached *after* JPEG conversion. The JPEG→queue work happens inside the "next upload allowed" window, so an expensive `fmt2jpg` runs even when the queue is already full (bad early-exit check). `xQueueSend` timeout (10 ticks) drops the image when full — no backoff.
- **Fix:** reorder: first decide whether the 60s window is open **and** a queue slot is free (check `uxQueueSpacesAvailable` or `xQueuePeek` before encoding), then convert, then enqueue.

### B3 — Queue is a fixed 5-item raw-memcpy queue with manual `free()` — error-prone RAII
`UploadJob` has a proper `~UploadJob()` (RAII) but `xQueueSend`/`xQueueReceive` operate by **raw memcpy**, which the code documents as bypassing C++ assignment/RAII — so every path manually `free()`s `job.data`/`job.metadata` (success path, full-queue drop path, destructor drain). Miss one path → leak. §5.3/§3.1 mandate RAII.
- **Fix:** use `std::queue<std::unique_ptr<UploadJob>>` guarded by a mutex (or a `StaticVector<UploadJob>` with move semantics) so ownership is explicit; keep `xQueue*` only if the task model must stay.

### B4 — `process_upload_sync()` runs a **blocking** `esp_http_client_perform()` on the main thread
`upload_task` is a FreeRTOS task (`xTaskCreate`), so uploads are off the main loop — **good**. But `config.timeout_ms = 5000` (correct per §3.4) and the HTTP perform blocks the worker for up to 5s. With a 5-item queue and a task that processes one item then loops, a slow server stall can back up the whole queue and each `collect_image` drops. No `defer()`/`set_timeout()` in play, no progress/abort for long stalls.
- **Fix:** add a per-job timeout/cancel path, and cap queue depth to what the 5s window can realistically drain (1/60s already paces it). Consider an `esp_http_client` chunked/open async handle instead of full sync `perform`.

### B5 — `last_upload_time_{0u - MIN_UPLOAD_INTERVAL_MS}` typo-style initialisation
`0u - 60000u` underflows to a large `uint32_t`, which is *intentional* ("first upload always permitted") per the comment — but `now - last_upload_time_` in `millis()` arithmetic can wrap across a 49-day uptime. With `last_upload_time_` initialised to a huge value the first subtraction is well-formed, but after that the 60s comparison treats a wrap-around uptime as "rate limited" spuriously.
- **Fix:** initialise to `0` and add an explicit `bool first_upload_` flag, or use a signed/int64 timestamp so wraparound is handled (§8.1 style).

### B6 — Metadata buffer size/logic
`job.metadata_len = strlen(metadata) + 1` then `heap_caps_malloc` and `strncpy(job.metadata, metadata, job.metadata_len)` — the `strncpy` correctly copies `metadata_len` bytes (includes NUL). Good, but `strlen` is called twice and metadata is re-passed to `process_upload_sync` as `char*` with a separate header. The `value[32]` buffer is `strncpy(..., sizeof-1)` + NUL — correct. Minor: repeated `strlen` on possibly-large metadata; cache length once.

### B7 — Missing `set_timeout`/non-blocking requirement for network (§3.4)
The uploader uses a dedicated task + blocking `perform()`. §3.4 says HTTP uploads must be NON-BLOCKING (`defer()`/`set_timeout()`), have timeout (✅ 5s), configurable auth (✅), and rate-limit 1/min (✅). The `esp_http_client` task approach satisfies "not on main thread" but is still blocking *in the worker* — acceptable, but makes B4 (stall backpressure) the real risk.

### B8 — `web_submit_switch` reads `switch->state` directly
`this->web_submit_switch_->state` is an unguarded read of a switch that can change under the main loop task; this is a TOCTOU-class cross-task read (§8.4). The switch state should be read under its owning mutex or via an atomic snapshot.
- **Fix:** snapshot `web_submit_switch_->state` into a local at call time (documenting that a torn switch read is benign for gating), or wrap in the component's lock.

### B9 — `esp_http_client_set_header` uses `raw_value.c_str()` — fine, but no `%s` format-safety pattern check
All `ESP_LOG*` calls use `"%s", user_input` (safe). ✅ The `X-Meter-Json`/`X-Meter-Value` headers are set from std::string — OK. No format-string vulnerabilities found. ✅ No `strcpy/sprintf` (uses `snprintf` for confidence `buf[32]` — ✅).

### B10 — Python pipeline lacks lint/type guards
`data_extractor/*.py` are part of the component. `pyproject.toml` configures ruff/flake8 at repo root — these Python files must pass `ruff check`/`ruff format --check`/flake8 (docstrings). Not reviewed line-by-line here — flag as a CI gate to run before merge.

---

## 3. Enhancements / new features

### E1 — Async/retry upload with backoff
Extend the worker to implement a bounded retry (e.g. 3 attempts, exponential backoff) on `esp_http_client_perform` failure instead of dropping immediately — feeds B4.

### E2 — Upload backpressure metrics
Expose (log or sensor) queue depth, drop count, and last HTTP status for observability — ties to B4/B9.

### E3 — Batch upload
For active-learning the 1/min rate is coarse; add configurable `upload_interval` and optionally batch multiple frames in one multipart POST to cut connection overhead.

### E4 — Grayscale/JPEG-aware JPEG re-encode skip
`collect_image()` already passes-through an already-JPEG frame (no re-encode) — good. Enhancement: when `format == "GRAYSCALE"` there is no JPEG input; `fmt2jpg` from raw grayscale needs a `PIXFORMAT_GRAYSCALE` branch so the frame can be encoded — current mapping defaults `PIXFORMAT_GRAYSCALE` only for unknown strings; verify a real grayscale `CameraImage` reaches `fmt2jpg` with correct `pix_fmt`.

### E5 — TLS/HTTPS support flag
`esp_http_client` with `https://` requires the TLS component; add a schema `use_tls` that conditionally references the `esp-tls` idf component (mirror `config.timeout`, `auth` already wired).

---

## 4. Test / repo compliance gaps

| Item | Status | Action |
|------|--------|--------|
| `cg.add_define("USE_DATA_COLLECTOR")` | present | mirror in `esphome/core/defines.h` |
| Build tests both ESP32/S3 | present | ✅ rename `esp32/esp32-s3.yaml` → `test.esp32-idf`/`test.esp32-s3-idf.yaml` (§9.1) — repo-wide |
| CRLF / whitespace | all files | repo-wide LF + whitespace sweep |
| Missing `#include <string.h>`? | indirect | verify compile via test |
| Multiple `strlen()` | minor | cache metadata length (B6) |

---

## 5. Priority ordering (bug-fix first)

1. B1/B2 — rate-limit window tied to *committed* upload + pre-encode queue check (silent-drop / wasted-encode bug).
2. B3 — replace raw-memcpy queue with `std::queue<unique_ptr<UploadJob>>` (RAII, §5.3 🔴).
3. B4 — bounded retry/backoff + stall handling.
4. B5 — uptime-wraparound-safe timestamp.
5. B8 — switch read snapshot.
6. B6 — metadata length caching.
7. E1–E5 enhancements after fixes land.
