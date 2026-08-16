#!/usr/bin/env bash
# Expand the Leo rover xacro into a single, package://-free URDF for the Isaac
# Sim URDF importer (which does not resolve xacro / $(find ...) / package://).
#
# Usage:  bash isaac/expand_leo_urdf.sh
set -euo pipefail

REPO=/home/lry/mars_navigation
LEO_COMMON="$REPO/ERC-Remote-Navigation-Sim/src/leo_common"
PKG="$LEO_COMMON/leo_description"
XACRO_IN="$PKG/urdf/leo.urdf.xacro"
OUT="$REPO/isaac/leo.urdf"

# Let $(find leo_description) resolve via rospkg.
export ROS_PACKAGE_PATH="$LEO_COMMON:${ROS_PACKAGE_PATH:-}"

mkdir -p "$(dirname "$OUT")"

# The ROS 2 xacro resolves $(find ...) via ament_index, which does not know this
# ROS 1 package.  Pre-rewrite the one $(find leo_description) to its absolute
# path, then expand xacro, then rewrite package:// mesh URLs to file://.
tmp="$(mktemp --suffix=.xacro)"
sed 's|$(find leo_description)|'"$PKG"'|g' "$XACRO_IN" > "$tmp"
xacro "$tmp" | sed "s|package://leo_description/|file://$PKG/|g" > "$OUT"
rm -f "$tmp"

echo "wrote $OUT"
echo "links:      $(grep -c '<link ' "$OUT")"
echo "joints:     $(grep -c '<joint ' "$OUT")"
echo "package:// remaining: $(grep -c 'package://' "$OUT" || true)"
