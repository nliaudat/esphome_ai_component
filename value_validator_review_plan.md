# value_validator — Review & Enhancement Plan

> Review against `.ai/instructions.md` (SINGLE SOURCE OF TRUTH), `.ai/instructions.yaml`,
> `.greptile/rules.md`, and prior review plans (`tflite_micro_helper`, `meter_reader_tflite`,
> `esp32_camera_utils`). Status: **Active** (shared validation engine, used_by all readers).

Scope: `value_validator.{h,cpp}`, `__init__.py`, `README.md`, `manifest.json`,
`value_validator_controls.yaml`, `value_validator_dial_controls.yaml`, + tests.

---

## 1. Summary

`value_validator` is a well-factored, standalone validation engine: `ReadingHistory` ring
buffer (bounded, RAII `unique_ptr`+`FreeDeleter`), PSRAM-preferred `psram_alloc` with
internal fallback, smart validation with self-correction/consensus, per-digit stability
rings, hallucination detection, `USE_ANALOG_READER` dial correction, optional diagnostic
sensors, and `persist_state` via NVS. No C-style casts, no `std::regex`, no `new`/`delete`
(raw heap use is RAII). The main findings are **the multi-instance persist-state collision**,
**lazy hot-path allocation**, and a **leading-zero digit-count loss** in the int setter.

---

## 2. Findings (priority per instructions.md §11)

### 🔴 BLOCKER

B1. **Multiple-instance `persist_state` NVS collision.**
    `setup()` uses `global_preferences->make_preference<int>(fnv1_hash("value_validator"))`
    — a **single fixed preference key** for *all* validator instances. The schema is
    `cv.ensure_list` (README advertises "Multiple Instances"), so two validators with
    `persist_state: true` share the **same NVS slot**: both `load()`/`save()` the same key,
    last-writer-wins corrupts the restored readings between readers (e.g. cold & hot meter).
    **Action:** E1 — per-instance key (incorporate `CONF_ID`/instance index, or derive a
    stable key from the instance), so each validator persists independently.

### 🟠 WARNING

W1. **Lazy heap allocation inside the hot `validate_reading()` path.**
    `validate_reading(span)` calls `ensure_digit_history_size()`, `ensure_last_valid_digits_size()`,
    and builds `std::vector` + `std::string` (`filtered_digit_string`, `std::to_string`) per call;
    it also reallocs `ensure_last_good_values_capacity()` on window change. For a fixed-size
    meter this is amortized (size-unchanged early-returns), but any digit-count change
    (including via `set_last_valid_reading`) triggers a **mid-loop `psram_alloc`**, violating
    §3.3/§4.2 "no heap allocation in loop() after setup()" and risking heap fragmentation/OOM.
    **Action:** E2 — pre-allocate in `setup()` for the *configured* max digit count, and use
    a stack/fixed `char[16]` for `filtered_digit_string` instead of `std::string`+`strtol`.

W2. **`set_last_valid_reading(int)` loses leading zeros / true digit count.**
    `std::to_string(value)` drops leading zeros: `set_last_valid_reading(50)` stores 2 digits,
    but a 5-digit meter may expect `00050`. `last_valid_digits_count_` then mismatches the real
    reading length, so `validate_reading(span)` logs "digit count changed…resetting" and
    **bypasses per-digit stability** for subsequent reads. The `(string)` overload preserves the
    count — callers should prefer it. **Action:** E3 — have the int setter accept a target
    digit count / callers pass the string form; or seed `last_valid_digits_` from the reading
    *length* context when known.

W3. **`ReadingHistory::get_median_within_ms` even-count median is biased-high.**
    For an even `size`, it returns `(*max_element(first half) + median_at(size/2))/2` —
    the true median would average the two middle elements (`values[size/2-1]` and
    `values[size/2]`). The current code uses the **max of the lower half** (a noisy
    upper-bound) instead of the exact lower-middle, so the "median" leans high and can
    occasionally pick a non-middle value. **Action:** E4 — compute the exact median
    (sort/`nth_element` both middle positions) or use `std::partial_sort` of the pair.

### 🟡 INFO

I1. **`is_hallucination_pattern` casts `long long val` to `int`** — fine for ≤9-digit
    readings; a 10-digit meter overflows the `int` (value up to `9_999_999_999` > INT_MAX),
    silently truncating the comparison `static_cast<int>(val) == last_valid_reading_`.
    **Action:** E5 — compare as `long long` / guard range.

I2. **`psram_alloc` pairs with `free()` in `FreeDeleter`.** `heap_caps_malloc(SPIRAM)` is
    freed with `free()` (works on ESP-IDF default heap in practice, but the strictly correct
    pairing is `heap_caps_free`). **Action:** E6 — use `heap_caps_free` in `FreeDeleter`, or
    allocate only via `heap_caps_malloc` + `heap_caps_free` consistently.

I3. **`ensure_capacity` floors to `MIN_HISTORY_ENTRIES=10`** even when
    `max_history_size_bytes` is small — capacity can exceed the configured byte budget.
    Minor; document. (E7)

I4. **`find_most_plausible_reading` uses the exact recent-mean then median** — the mean of
    `int` readings can be non-representable/rounding to a plausible value; acceptable, but
    the mean path can return a value that never actually occurred. INFO (E8 — prefer median
    first, mean only if median implausible).

I5. **`set_smart_validation_window` reallocs the good-values ring** — called in
    `to_code`→`setup()` (fine), but if ever called at runtime reallocates mid-loop. Note E2
    covers this (pre-allocate to max window).

I6. **Per-digit `filtered_digit_string` + `strtol` reparse loses leading zeros then
    re-pads** — net correct, but `strtol` on a 10+ digit string is range-checked (good).
    Covered by E2.

---

## 3. Enhancement Plan (ordered)

### E1 — Per-instance persistent-state key 🔴 ✅ DONE
<br>Fixed: `__init__.py` emits a per-instance SHA-256 salt from the YAML id;
<br>`setup()` XORs it into the NVS preference key.
**What:** derive a stable preference key per validator instance, e.g. incorporate the
component instance id (or an index/counter emitted by `__init__.py`) into the
`fnv1_hash` input.
**Why:** B1 — two persisted validators currently share one NVS slot and corrupt each other.
**How:** `__init__.py` passes a per-instance salt (e.g. `fnv1_hash("value_validator_"+id)`),
or `setup()` derives from the assigned `Component` reference/pointer — pick the codegen-emitted
salt approach (deterministic across builds).

### E2 — Pre-allocate in setup; stack-buffer the filter string 🟠
**What:**
- In `setup()`, call `ensure_last_valid_digits_size()` and `ensure_digit_history_size()` for
  the *configured/expected* max digits (config option `expected_digits`, or defer once on
  first `set_last_valid_reading`/string path).
- Replace `std::string filtered_digit_string` + `std::to_string` + `strtol` with a
  fixed `char[24]` buffer + manual append + `strtol`, removing per-call heap churn.
- Keep the `ensure_*` early-returns but guarantee no allocation occurs after the once-only
  lazy init.
**How:** follows §3.3/§4.2 (allocate in `setup()`, no heap in `loop()`).

### E3 — Preserve digit count in `set_last_valid_reading(int)` 🟠 ✅ DONE
<br>Fixed: added `set_last_valid_reading(int, size_t num_digits)` overload that
<br>zero-pads to the fixed width; the int-only setter delegates with 0 (infer).
**What:** add an overload/flag so the int setter can retain leading-zero length, or document
& require the string form for fixed-width meters; seed `last_valid_digits_` consistently.
**Why:** W2 — int setter truncates the digit count, breaking per-digit stability.
**How:** add `set_last_valid_reading(const std::string&)` as the canonical path (already
exists); make `set_last_valid_reading(int)` accept an optional `num_digits` or store from
the last *real* reading length.

### E4 — Exact even-count median in `get_median_within_ms` 🟠 ✅ DONE
<br>Fixed: computes the average of the two true middle elements (nth_element twice).
**What:** for even `size`, average the two middle elements exactly (use
`nth_element` to find both `size/2-1` and `size/2`).
**Why:** W3 — biased-high median can skew consensus/self-correction decisions.
**How:** mechanical (`std::nth_element` twice, or `partial_sort` of middle pair).

### E5 — `is_hallucination_pattern` int-overflow-safe compare 🟡 ✅ DONE
<br>Fixed: compares `val` as `long long` against `last_valid_reading_` (no int truncation).
**What:** compare `val` (already `long long`) against `last_valid_reading_` as `long long`;
only cast to `int` after a range check.
**Why:** I1 — 10-digit readings overflow `int`.
**How:** `if (val == static_cast<long long>(this->last_valid_reading_))`.

### E6 — Correct `heap_caps_free` pairing in `FreeDeleter` 🟡 ✅ DONE
<br>Fixed: `FreeDeleter::operator()` now calls `heap_caps_free` (matching `psram_alloc`'s
<br>`heap_caps_malloc`); added `<esp_heap_caps.h>` include.
**What:** `FreeDeleter::operator()` calls `heap_caps_free` when the buffer came from
`heap_caps_malloc`, `free` otherwise (or use one allocator consistently).
**Why:** I2 — alloc/free API pairing correctness.
**How:** `psram_alloc` returns from either path; tag the origin or always use
`heap_caps_free` (safe for IDF default heap).

### E7 — Document the history-capacity floor 🟡
**What:** note `MIN_HISTORY_ENTRIES=10` flooring; optionally allow 0 in the schema and let
the floor apply only when history is enabled.
**Why:** I3 — small-budget configs get a larger ring than asked.

### E8 — Prefer median over mean in `find_most_plausible_reading` 🟡
**What:** try the **median** of `recent_readings` first, then the mean, then last-plausible.
**Why:** I4 — mean can return a value that never occurred and may round to a confusing
reading; median is more robust for outlier-heavy reads.
**How:** reorder the branches (median → mean → scan recent).

### E9 — Expand tests (multi-instance + leading-zero) 🟡
**What:** add an `esp32-s3.yaml` (or common.yaml) test with **two** validators both
`persist_state: true` (regression for E1), and a fixed-width 5-digit meter path with a
`00050`-style reading (regression for E3). Keep dual-target §9.1.
**Why:** B1/W2 are currently untested; the existing tests are minimal (esp32/common use
defaults; esp32-s3 turns on strict/persist but only one instance).

---

## 4. Sequencing
1. **E1 (B1, multi-instance NVS collision)** — the only real blocker.
2. **E3 (leading-zero), E2 (hot-path alloc), E4 (median)** — correctness/robustness.
3. **E5–E8 (INFO hardening) + E9 (tests)** — low risk.

## 5. Do NOT change
- The `ReadingHistory` bounded ring + RAII `unique_ptr` design and PSRAM preference.
- The smart-validation/self-correction consensus and per-digit stability semantics.
- The `USE_ANALOG_READER` dial-correction logic (correct: converts dial fraction, subtracts 1
  on wrap, keeps raw on solid digit; middle-zone no-op).
- The schema `cv.ensure_list` multi-instance support (keep — but fix the NVS key via E1).
- `strtol`-based overflow-safe parsing (§8.1 patterns) — keep those, replace only the heap
  `std::string` accumulation (E2).
