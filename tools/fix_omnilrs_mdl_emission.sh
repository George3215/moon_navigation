#!/usr/bin/env bash
# Idempotently disable the RED emission leak in OmniLRS MDL materials.
#
# OmniLRS MDL files declare `enable_emission: false` yet still render a bright
# pink/red glow (emissive_color=(1,0.1,0.1) * emissive_intensity=40) on shadowed
# faces in Isaac Sim 5.1.0 — the MDL runtime ignores the enable_emission gate.
# Patching `emissive_intensity` to 0 kills the leak while keeping the exact
# OmniLRS diffuse textures (gray gravel / regolith), so the scene stays gray.
#
# Usage:  bash tools/fix_omnilrs_mdl_emission.sh [/path/to/OmniLRS]
set -euo pipefail

ROOT="${1:-${OMNILRS_ROOT:-/home/lry/OmniLRS}}"
TEX="$ROOT/assets/Textures"
FILES=(
  "GravelStones.mdl"
  "rock_boulder_dry.mdl"
  "LunarRegolith8k.mdl"
  "seaside_rock_2k.mdl"
)

patched=0
for f in "${FILES[@]}"; do
  p="$TEX/$f"
  if [ ! -f "$p" ]; then
    echo "skip (missing): $p"
    continue
  fi
  if grep -q "emissive_intensity: 0" "$p"; then
    echo "already patched: $p"
    continue
  fi
  sed -i 's/emissive_intensity: 40\.0*[f]*,/emissive_intensity: 0.f,/' "$p"
  echo "patched: $p"
  patched=$((patched + 1))
done

if [ "$patched" -gt 0 ]; then
  echo "OK: $patched OmniLRS MDL file(s) patched. Red emission leak disabled."
else
  echo "OK: no OmniLRS MDL files needed patching."
fi
