# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 2.0.x   | :white_check_mark: |
| < 2.0   | :x:                |

## Reporting a Vulnerability

If you discover a security vulnerability in this project, please report it privately via GitHub Security Advisories:

1. Go to https://github.com/nliaudat/esphome_ai_component/security/advisories
2. Click "New draft security advisory"
3. Provide a detailed description of the vulnerability

Please do **not** report security vulnerabilities via public GitHub issues.

## What to Expect

- **Acknowledgment** within 48 hours
- **Initial triage** within 5 business days
- **Fix timeline** communicated after triage

## Scope

This policy covers all components in this repository. For vulnerabilities in ESPHome core or the TensorFlow Lite Micro runtime, please report to those respective projects.

## Security Considerations for This Project

- **Camera images** are processed locally on-device and never transmitted by default
- **Data collection** (active learning) sends only low-confidence crops to a user-configured server — review your network configuration if enabling this
- **OTA updates** are secured via ESPHome's built-in encryption when configured
- **Model files** (`.tflite`) are validated with CRC32 checksums on every load to prevent corruption