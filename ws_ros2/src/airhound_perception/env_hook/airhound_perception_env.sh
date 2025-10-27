# Environment hook for airhound_perception
# This file is sourced automatically when you `source install/setup.bash`
# after building the workspace. It attempts to:
#   1. Expose the project virtual environment (if specified) to ROS Python path.
#   2. Ensure TensorRT Python bindings (from system) remain visible when using a venv.
#   3. Set safe Ultralytics export environment variables on Jetson.
#   4. Optionally surface the YOLO engine/model directory via AIRHOUND_MODEL_DIR.
#
# Usage:
#   export AIRHOUND_VENV=/absolute/path/to/clean_workspace/venv
#   source install/setup.bash
#
# If AIRHOUND_VENV is NOT set, this hook does nothing (silent no‑op).
#
# Recommended (to suppress colcon scanning the venv):
#   touch /absolute/path/to/clean_workspace/venv/COLCON_IGNORE
#
# Disable any step by exporting:
#   AIRHOUND_NO_VENV=1              -> skip adding venv site-packages
#   AIRHOUND_NO_TRT=1               -> skip TensorRT path injection
#   AIRHOUND_NO_ULTRA_FLAGS=1       -> skip Ultralytics requirement suppression
#
# Detected exports (if venv provided):
#   PYTHONPATH += <venv>/lib/python3.10/site-packages
#   (fallback also checks lib64)
#
# Jetson conveniences:
#   Sets ULTRALYTICS_IGNORE_REQUIREMENTS=1 (avoids futile onnxruntime-gpu / tensorrt-cu12 installs)
#
# Idempotent: re-sourcing does not duplicate paths.

_airhound_hook_log() {
  # Avoid noisy logs unless explicitly requested
  if [ -n "${AIRHOUND_ENV_DEBUG:-}" ]; then
    echo "[airhound_perception.env] $*"
  fi
}

# 1. Virtual environment site-packages exposure
if [ -z "${AIRHOUND_NO_VENV:-}" ] && [ -n "${AIRHOUND_VENV:-}" ] && [ -d "${AIRHOUND_VENV}" ]; then
  _airhound_hook_log "AIRHOUND_VENV detected at: ${AIRHOUND_VENV}"

  # Prefer python3.10 path; fallback to lib64 if present
  _PY_SITE="${AIRHOUND_VENV}/lib/python3.10/site-packages"
  _PY_SITE64="${AIRHOUND_VENV}/lib64/python3.10/site-packages"

  if [ -d "${_PY_SITE}" ]; then
    case ":$PYTHONPATH:" in
      *":${_PY_SITE}:"*) : ;; # already present
      *) export PYTHONPATH="${_PY_SITE}:${PYTHONPATH:-}"; _airhound_hook_log "Added venv site-packages: ${_PY_SITE}" ;;
    esac
  fi
  if [ -d "${_PY_SITE64}" ]; then
    case ":$PYTHONPATH:" in
      *":${_PY_SITE64}:"*) : ;;
      *) export PYTHONPATH="${_PY_SITE64}:${PYTHONPATH:-}"; _airhound_hook_log "Added venv site-packages (lib64): ${_PY_SITE64}" ;;
    esac
  fi

  # 2. Ensure system dist-packages visible for TensorRT if venv was created without --system-site-packages
  if [ -z "${AIRHOUND_NO_TRT:-}" ] && [ -f /etc/nv_tegra_release ]; then
    # Typical system dist-packages path
    _SYS_DIST="/usr/lib/python3.10/dist-packages"
    if [ -d "${_SYS_DIST}" ]; then
      case ":$PYTHONPATH:" in
        *":${_SYS_DIST}:"*) : ;;
        *) export PYTHONPATH="${_SYS_DIST}:${PYTHONPATH}"; _airhound_hook_log "Added system dist-packages for TensorRT: ${_SYS_DIST}" ;;
      esac
    fi
  fi
fi

# 3. Jetson Ultralytics export convenience
if [ -z "${AIRHOUND_NO_ULTRA_FLAGS:-}" ] && [ -f /etc/nv_tegra_release ]; then
  # Avoid repeated re-installs of unsupported wheels on Jetson
  export ULTRALYTICS_IGNORE_REQUIREMENTS=1
  _airhound_hook_log "Set ULTRALYTICS_IGNORE_REQUIREMENTS=1 (Jetson)"
fi

# 4. Model directory hint (helpful for downstream scripts)
# Attempt to guess the source share directory (this hook lives in install space)
# If user sets AIRHOUND_MODEL_DIR externally, do not override.
if [ -z "${AIRHOUND_MODEL_DIR:-}" ]; then
  # When installed, this hook is under: <prefix>/share/airhound_perception/env_hook/
  _HOOK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd)"
  _PKG_SHARE="$(cd "${_HOOK_DIR}/.." 2>/dev/null && pwd)"
  # Not all models are installed/copied; only set if a models/ directory is found adjacent to workspace root guess.
  # Users typically run from source; in that case leave it unset unless discovered.
  if [ -d "${_PKG_SHARE}/models" ]; then
    export AIRHOUND_MODEL_DIR="${_PKG_SHARE}/models"
    _airhound_hook_log "Detected AIRHOUND_MODEL_DIR=${AIRHOUND_MODEL_DIR}"
  fi
fi

# 5. Optional: warn once if Ultralytics still missing when detector node will need it
if [ -n "${AIRHOUND_ENV_DEBUG:-}" ]; then
  python3 - <<'PY' 2>/dev/null || true
try:
    import ultralytics
except Exception as e:
    print("[airhound_perception.env] WARNING: 'ultralytics' not importable in current environment.")
PY
fi

# Done
return 0 2>/dev/null || true
