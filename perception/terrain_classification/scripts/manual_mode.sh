#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../../tools/python_env.sh"
cd "$SCRIPT_DIR"
"$PYTHON_BIN" mode_handler.py
