#!/usr/bin/env bash
# verify/check_topics.sh — scripted acceptance / regression gate.
# Asserts the verified topic contract is ALIVE on the sim's DDS (domain 0).
# Extra required topics can be passed as args, e.g.:
#   ./verify/check_topics.sh /oakd/rgb/image_raw /oakd/stereo/image_raw /oakd/points
#
# RUN THIS WHILE the sim is up. It does NOT spawn a sim (a 2nd Isaac instance
# violates the one-instance rule). Run it inside an `isaac-ros` shell, or keep the
# env block below reconciled with whatever `isaac-ros` actually sets.
set -uo pipefail

# --- match the sim's DDS (reconciled with the real `isaac-ros` alias) ---
# isaac-ros sets: unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT; ROS_DOMAIN_ID=0;
# ROS_LOCALHOST_ONLY=1; RMW_IMPLEMENTATION=rmw_fastrtps_cpp; ros2 daemon stop.
# All of these must match or this checker's DDS participant won't discover the sim.
[ -z "${AMENT_PREFIX_PATH:-}" ] && source /opt/ros/humble/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT 2>/dev/null || true   # real robot uses these; sim must not
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop >/dev/null 2>&1 || true            # drop stale graph after any domain switch

BASE_TOPICS=(/clock /cmd_vel /odom /tf /scan)
EXTRA_TOPICS=("$@")
HZ_TOPICS=(/scan /odom "$@")
HZ_TIMEOUT=15

fail() { echo "FAIL: $*"; exit 1; }

echo "[check_topics] domain $ROS_DOMAIN_ID — verifying contract..."
LIST="$(timeout 10 ros2 topic list 2>/dev/null)" || fail "ros2 topic list timed out (is the sim up? daemon stale?)"

for t in "${BASE_TOPICS[@]}" "${EXTRA_TOPICS[@]}"; do
  printf '%s\n' "$LIST" | grep -qx "$t" || fail "missing topic: $t"
done

for t in "${HZ_TOPICS[@]}"; do
  [ -z "$t" ] && continue
  # NB: `grep -q`/`-m1` exits on first match and SIGPIPE-kills `ros2 topic hz`.
  # Under `set -o pipefail` that upstream 120/141 becomes the pipeline status, so
  # `if ! ... | grep -q` wrongly fails a HEALTHY topic. Capture the line in a
  # subshell instead and test it: empty (silent topic) still fails, as intended.
  rate="$(timeout "$HZ_TIMEOUT" ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate')"
  [ -n "$rate" ] || fail "$t not publishing (no rate within ${HZ_TIMEOUT}s)"
done

echo "PASS: required topics present and ${HZ_TOPICS[*]} publishing."
exit 0
