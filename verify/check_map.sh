#!/usr/bin/env bash
# verify/check_map.sh — Phase-3 scripted acceptance / lidar+SLAM regression guard.
#
# Asserts (a) the verified base contract is still alive AND /scan is UNCHANGED
# (frame rplidar_link, 360 deg, filtered at SCAN_MIN_RANGE), and (b) a saved map
# exists, is non-empty, and contains real occupancy (free + occupied cells).
#
# Run WHILE the sim is up (it does NOT spawn a sim). Build the map first with
# SLAM + verify/scripted_teleop.py + map_saver_cli (see README Phase 3).
#   ./verify/check_map.sh [map_basename]     # default: maps/A-1_phase3_map
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
MAP="${1:-$HERE/../maps/A-1_phase3_map}"
SCAN_MIN_RANGE="${SCAN_MIN_RANGE:-0.32}"

# match the sim's DDS (same block as check_topics.sh)
[ -z "${AMENT_PREFIX_PATH:-}" ] && source /opt/ros/humble/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT 2>/dev/null || true
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop >/dev/null 2>&1 || true

fail() { echo "FAIL: $*"; exit 1; }

# (a) base contract still green
echo "[check_map] base contract regression..."
"$HERE/check_topics.sh" >/dev/null 2>&1 || fail "base contract (check_topics.sh) is RED"

# (a') /scan unchanged: frame rplidar_link + filter at SCAN_MIN_RANGE
SCAN="$(timeout 15 ros2 topic echo --once /scan 2>/dev/null)" || fail "/scan echo timed out"
printf '%s\n' "$SCAN" | grep -q "frame_id: rplidar_link" || fail "/scan frame_id is not rplidar_link"
RMIN="$(printf '%s\n' "$SCAN" | grep -m1 'range_min:' | awk '{print $2}')"
awk "BEGIN{exit !($RMIN >= $SCAN_MIN_RANGE - 1e-4)}" \
  || fail "/scan range_min ($RMIN) < SCAN_MIN_RANGE ($SCAN_MIN_RANGE) — filter regressed"

# (b) saved map exists, non-empty, has real occupancy content
[ -s "${MAP}.pgm" ]  || fail "map ${MAP}.pgm missing or empty"
[ -s "${MAP}.yaml" ] || fail "map ${MAP}.yaml missing or empty"
python3 - "$MAP" <<'PY' || exit 1
import sys, numpy as np
from PIL import Image
m = sys.argv[1]
im = np.array(Image.open(m + ".pgm"))
free = int((im >= 250).sum()); occ = int((im <= 5).sum()); tot = im.size
print(f"[check_map] map {im.shape}: free {100*free/tot:.0f}% occupied {100*occ/tot:.0f}%")
if free < 50 or occ < 10:
    print("FAIL: map has no real occupancy (free<50 or occupied<10 cells)", file=sys.stderr)
    sys.exit(1)
PY

echo "PASS: base contract green, /scan frame+filter intact, map ${MAP}.pgm/.yaml saved + valid."
exit 0
