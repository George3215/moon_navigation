#!/bin/bash
# =============================================================
# Python Environment Configuration
# =============================================================
# All Python launch scripts source this file to locate the
# correct Python interpreter. Set PYTHON_BIN to the absolute
# path of your conda / virtualenv / system Python.
#
# Usage (in any launch .sh script):
#   SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#   source "$(dirname "$SCRIPT_DIR")/../../tools/python_env.sh"
#   cd "$SCRIPT_DIR"
#   "$PYTHON_BIN" my_script.py
# =============================================================

# Example:
#   PYTHON_BIN="$HOME/miniconda3/envs/mars_nav/bin/python"
# You can also export PYTHON_BIN before launching the stack.

# PYTHON_BIN="${PYTHON_BIN:-python3}"
PYTHON_BIN="/home/chengsn/anaconda3/envs/torch_38/bin/python"

if [[ "$PYTHON_BIN" == */* ]]; then
    RESOLVED_PYTHON_BIN="$PYTHON_BIN"
else
    RESOLVED_PYTHON_BIN="$(command -v "$PYTHON_BIN" 2>/dev/null || true)"
fi

# Sanity check
if [ -z "$RESOLVED_PYTHON_BIN" ] || [ ! -x "$RESOLVED_PYTHON_BIN" ]; then
    echo "[ERROR] Python interpreter not found: $PYTHON_BIN"
    echo "Please install the dependencies from requirements.txt, then edit tools/python_env.sh or set the PYTHON_BIN environment variable."
    exit 1
fi

PYTHON_BIN="$RESOLVED_PYTHON_BIN"
