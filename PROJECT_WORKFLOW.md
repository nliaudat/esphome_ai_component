# ESP32-CAM Meter Reader - Project Workflow

**Last Updated:** 2026-03-22
**Branch:** main-clean

## Project Structure

This repository contains ESPHome configurations for ESP32-CAM based water meter reading using AI/ML.

```
esp32-cam/
├── test-ai-reader.yaml          # Main config (ESP-IDF, remote component)
├── test-ai-reader-local.yaml    # Development config (local components)
├── dig-class100-0168_s2_q.tflite # Model file (226KB - fits in 4MB flash)
├── boards/                      # Board definition files
├── models/                      # Additional model files
├── secrets.yaml                 # WiFi/API secrets (gitignored!)
├── .gitignore                   # Git exclusions
└── PROJECT_WORKFLOW.md          # This file
```

## Git Workflow

### Repository Setup

**This repo tracks:**
- ESPHome YAML configurations
- Model files (.tflite)
- Board definitions
- Documentation

**This repo does NOT track:**
- C++ component source code (use remote GitHub repo)
- Build artifacts (.esphome/, .pioenvs/)
- Secrets (secrets.yaml)

### Branches

| Branch | Purpose |
|--------|---------|
| `main-clean` | Clean configs only, uses remote components |
| `esphome-ai-component-fix` | Legacy branch with local components |
| `dev/*` | Feature branches for component development |

### Daily Use (Remote Component)

```yaml
# test-ai-reader.yaml
external_components:
  - source: github://nliaudat/esphome_ai_component@main
    components: 
      - tflite_micro_helper
      - esp32_camera_utils
      - flash_light_controller
      - meter_reader_tflite
```

### Component Development (Local)

When you need to modify C++ components:

```bash
# 1. Create development branch from upstream
GIT_MASTER=1 git fetch upstream
GIT_MASTER=1 git checkout -b dev/my-feature upstream/main

# 2. Copy components temporarily
GIT_MASTER=1 git checkout upstream/main -- components/

# 3. Edit components/...

# 4. Test with local path in YAML:
# external_components:
#   - source:
#       type: local
#       path: components
#     components: [meter_reader_tflite]

# 5. Push to your fork and create PR
GIT_MASTER=1 git add components/
GIT_MASTER=1 git commit -m "feat: description"
GIT_MASTER=1 git push fork dev/my-feature
gh pr create --repo nliaudat/esphome_ai_component
```

### Pushing Component Changes to Your Fork

After testing locally, push changes to your fork so others can use them:

```bash
# 1. Go to your component fork
GIT_MASTER=1 cd ~/src/esphome_ai_component_fork

# 2. Create/checkout your branch
GIT_MASTER=1 git checkout -b mikaabra origin/main

# 3. Copy changes from local components
GIT_MASTER=1 cp ~/src/esp32-cam/components/meter_reader_tflite/model_config.h \
              components/meter_reader_tflite/model_config.h

# 4. Commit and push
GIT_MASTER=1 git add components/meter_reader_tflite/model_config.h
GIT_MASTER=1 git commit -m "feat: add model config for dig-class100-0168_s2_q"
GIT_MASTER=1 git push origin mikaabra
```

Then update your YAML to use your fork:
```yaml
external_components:
  - source: github://mikaabra/esphome_ai_component@mikaabra
    components: 
      - meter_reader_tflite
```

## Key Configurations

### test-ai-reader.yaml (Production)
- **Framework:** ESP-IDF (more memory efficient)
- **Component Source:** GitHub remote
- **Model:** dig-class100-0168_s2_q.tflite (226KB)
- **Flash Usage:** ~95% (fits in 4MB)

### test-ai-reader-local.yaml (Development)
- **Framework:** ESP-IDF
- **Component Source:** Local `components/` directory
- **Use for:** Testing C++ changes before submitting PR

## Model Files

| Model | Size | Flash Usage |
|-------|------|-------------|
| dig-class100-0168_s2_q.tflite | 226KB | ✅ Fits (95%) |
| dig-class100-0173-s2-q.tflite | 305KB | ❌ Too big (104%) |

**Always use 0168 model for ESP32-CAM with 4MB flash.**

## Framework Differences

| Framework | Flash Overhead | Use Case |
|-----------|----------------|----------|
| Arduino | ~200KB larger | Legacy, simpler setup |
| ESP-IDF | More efficient | Production, recommended |

## Common Commands

```bash
# Compile and flash
esphome run test-ai-reader.yaml

# Clean build
esphome clean test-ai-reader.yaml

# Check config
esphome config test-ai-reader.yaml

# Git workflow
GIT_MASTER=1 git status
GIT_MASTER=1 git add <files>
GIT_MASTER=1 git commit -m "message"
GIT_MASTER=1 git push origin main-clean
```

## Important Notes

1. **Never commit secrets.yaml** - It's gitignored for security
2. **Keep components/ untracked** - Use remote GitHub repo for daily use
3. **ESP-IDF is required** - Arduino framework causes flash overflow
4. **Use old model (0168)** - New model (0173) is too large

## Upstream Repos

- **Main component:** https://github.com/nliaudat/esphome_ai_component
- **Your fork:** https://github.com/mikaabra/esphome_ai_component
- **Issue tracker:** https://github.com/jomjol/AI-on-the-edge-device

## Flash Size Troubleshooting

If flash size exceeds 100%:
1. ✅ Check you're using ESP-IDF framework
2. ✅ Check you're using dig-class100-0168 model (not 0173)
3. ✅ Remove local components/ directory
4. ✅ Use remote component from GitHub

## Session Continuity

When starting a new session, load this file:

```
Read /Users/mikaelabrahamsson/src/esp32-cam/PROJECT_WORKFLOW.md
```

This will provide context about:
- Current branch (main-clean)
- Project structure
- Git workflow
- Known issues and solutions
