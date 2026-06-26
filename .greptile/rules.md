# Greptile Rules — esphome_ai_component

**This project MUST comply with `.ai/instructions.yaml` as the token-optimized rule source.**
**Full details: `.ai/instructions.md` (SINGLE SOURCE OF TRUTH).**

## Mandatory: Load `.ai/instructions.yaml` Before Analysis
Before reading or modifying any code, load `.ai/instructions.yaml` for the machine-readable
rule set. Fall back to `.ai/instructions.md` for context and code examples.
These files override generic C++/ESPHome advice.

## ESP32 Patterns — DO NOT FLAG (true positives on this platform)
The following patterns are flagged as defects by static analysis but are CORRECT on ESP32.
Do NOT suggest changes:

| Pattern ID | Description | Reason |
|------------|-------------|--------|
| SUP-QUEUE-FREE | `xQueueReceive` + manual free+nullify outside while() | xQueueReceive raw memcpy bypasses C++ RAII |
| SUP-JPEG-DELETER | `jpeg_dec_handle_t` custom deleter struct | ESP-IDF handle type is void* |
| SUP-FMODF | `fmodf(angle, 360.0f)` for normalization | O(1) safe vs while-loop watchdog risk |
| SUP-MODEL-DIRS | Static dir list without exists() filter | discover_models() checks per-scan |
| SUP-TYPE-CAST | Raw int (0/1) for type mapping | Named enums may differ across components |
| SUP-VALIDATE-GATE | max_consecutive_rejections gating | Allows rollovers, rejects glitches |

## Key Non-Negotiables (from .ai/instructions.yaml)
- **§7.2/§10**: `this->` prefix on ALL class member access
- **§7.4**: `constexpr` over `#define` for constants (except conditional compilation)
- **§10**: `static_cast<>` instead of C-style casts
- **§5**: RAII (`unique_ptr`), `TrackedBuffer` for camera, no heap alloc in `loop()`
- **§3.1**: Single `std::atomic<FrameState>` state machine (NO TOCTOU CWE-367)
- **§10**: No `std::regex`, no blocking network, no `delay()` in `loop()`
- **§12.4**: `legacy_meter_reader_tflite` — EXCLUDED from review (frozen/deprecated)
- **§12.5**: Dual-target awareness (ESP32-S3 speed + classic ESP32 memory)
- **§8.1**: Integer overflow checks in pointer arithmetic (CVE-2026-23833 pattern)
- **§12.10**: No unauthorized commits — present changes as proposals first
- **License**: Apache 2.0 OR MIT (dual license)

## Review Priority (instructions.md §11)
- 🔴 BLOCKER: Memory leaks, buffer overflows, camera buffer mismanagement, TOCTOU
- 🟠 WARNING: Performance regression, missing validation, heap alloc in `loop()`
- 🟡 INFO: Const correctness, naming, dead code, include-what-you-use

## Reference
- `.ai/instructions.yaml` — token-optimized rule database (for automated queries)
- `.ai/instructions.md` — full human-readable rules (for context/examples)
- `.clinerules/rules.yaml` — Cline rule database (synced from instructions.yaml)
