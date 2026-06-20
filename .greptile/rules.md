# Greptile Rules — esphome_ai_component

**This project MUST comply with `.ai/instructions.md` as the SINGLE SOURCE OF TRUTH.**

## Mandatory: Read `.ai/instructions.md` Before Any Analysis
Before reading or modifying any code, load and comply with `.ai/instructions.md` in full.
This file overrides generic C++/ESPHome advice.

## Key Non-Negotiables (from instructions.md)
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
- **License**: CC-BY-NC-SA — no commercial use

## Review Priority (instructions.md §11)
- 🔴 BLOCKER: Memory leaks, buffer overflows, camera buffer mismanagement, TOCTOU
- 🟠 WARNING: Performance regression, missing validation, heap alloc in `loop()`
- 🟡 INFO: Const correctness, naming, dead code, include-what-you-use
