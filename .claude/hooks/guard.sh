#!/usr/bin/env bash
# .claude/hooks/guard.sh — PreToolUse(Bash) safety net for unattended runs.
# Reads the tool-call JSON on stdin and DENIES commands that could break the
# verified sim, the real-robot config, or git history. Requires `jq`.
# Project hooks require approval on first use — approve it once when prompted.
set -uo pipefail

INPUT="$(cat)"
# Extract the command. Prefer jq, fall back to python3, then to the raw payload.
if command -v jq >/dev/null 2>&1; then
  CMD="$(printf '%s' "$INPUT" | jq -r '.tool_input.command // ""' 2>/dev/null)"
elif command -v python3 >/dev/null 2>&1; then
  CMD="$(printf '%s' "$INPUT" | python3 -c 'import sys,json; print(json.load(sys.stdin).get("tool_input",{}).get("command",""))' 2>/dev/null)"
else
  CMD="$INPUT"
fi

deny() {
  # JSON deny + exit 2 blocks the tool call (works even under bypass modes).
  printf '{"hookSpecificOutput":{"hookEventName":"PreToolUse","permissionDecision":"deny","permissionDecisionReason":"%s"}}' "$1"
  exit 2
}

# --- never touch the real-robot config (sim is domain 0, no discovery server) ---
printf '%s' "$CMD" | grep -Eq 'ROS_DOMAIN_ID=[^0[:space:]]'   && deny "Sim is domain 0 only. Do not set ROS_DOMAIN_ID to the real robot."
printf '%s' "$CMD" | grep -Eq 'ROS_DISCOVERY_SERVER'          && deny "The sim runs with NO discovery server; that is the real-robot config."

# --- protect the USD + its backup, and avoid blind destructive deletes ---
printf '%s' "$CMD" | grep -Eq 'rm[^|]*turtlebot4\.usd\.bak'   && deny "Do not delete the USD backup."
printf '%s' "$CMD" | grep -Eq 'rm[[:space:]]+-[a-zA-Z]*r[a-zA-Z]*f|rm[[:space:]]+-[a-zA-Z]*f[a-zA-Z]*r' && deny "rm -rf is blocked unattended. Be specific or move to a trash dir."

# --- protect git history ---
printf '%s' "$CMD" | grep -Eq 'git[[:space:]]+push[[:space:]]+(-f|--force)' && deny "No force-push during unattended runs."
printf '%s' "$CMD" | grep -Eq 'git[[:space:]]+reset[[:space:]]+--hard'      && deny "No hard reset. Use /rewind or revert a specific commit."
printf '%s' "$CMD" | grep -Eq 'git[[:space:]]+clean[[:space:]]+-[a-z]*f'    && deny "git clean -f is blocked; it can wipe untracked work."

exit 0
