# GOAL (UNATTENDED) — Phases 1–3 only, self-verifying

Run this **autonomously** while I'm away. Phases 1–3 only. **Phase 4 (the HMI GUI extension) is
DEFERRED — do NOT attempt it unattended; it needs a human at the screen.** Standing rules are in
`CLAUDE.md` (domain 0, render-on-even-when-headless, one Isaac instance, real-TB4 parity, back up
the USD). Read those first. The full interactive spec is `CLAUDE_CODE_GOAL.md` — same Phase 1–3
content; this file changes only *how you proceed* (no waiting on me) and *how you verify* (scripts,
not my eyes).

## First, establish a green baseline (do this before any phase)
1. Read `README.md`, `COMMANDS.md`, and the real `isaac-ros` alias. **Reconcile
   `verify/check_topics.sh` with whatever env `isaac-ros` sets** (domain 0, discovery server
   unset, daemon reset). This script is your acceptance gate — it must match the sim's DDS.
2. Create a work branch: `git checkout -b feat/oakd-phases-1-3`. Record it in `STATUS.md`.
3. Bring the sim up **headless with rendering on**, as a **background task** (never spawn a
   second Isaac instance — reuse this one for all checks).
4. Run `./verify/check_topics.sh`. It must PASS (the verified `/scan /odom /tf /clock /cmd_vel`
   contract is alive). Commit this as the last-known-good baseline. If it can't pass, STOP and
   write why in `STATUS.md` — do not start Phase 1 on a red baseline.

## How to proceed at every step (the replaced gate)
For each unit of work:
1. Make the change (small + scripted; back up the USD before binary edits).
2. Run the **scripted acceptance** for that phase (below) against the running sim.
3. **PASS** → `git commit` on the branch, append a one-line result to `STATUS.md`, continue.
4. **FAIL** → revert to the last green commit (`/rewind` or `git revert`/`git checkout -- <file>`),
   append the failure + your hypothesis to `STATUS.md`, and **STOP**. Do not pile more changes on
   a red state hoping it resolves. Leave it for me.
Never weaken or delete a check to make it pass. Never edit the real-robot config to "fix" DDS.

## Scripted acceptance per phase (no human eyes)
- **Phase 1 — OAK-D RGB+depth+points.** After adding the camera, extend the gate:
  `./verify/check_topics.sh /oakd/rgb/image_raw /oakd/rgb/camera_info /oakd/stereo/image_raw /oakd/points`
  Also assert `ros2 topic echo --once /oakd/rgb/camera_info` has the right frame + non-empty
  intrinsics, and save one RGB frame + one depth frame to `verify/artifacts/` (rosbag or
  image_saver) **for me to eyeball later** — do not try to judge image quality yourself.
- **Phase 2 — detection + spatial.** With the detection node running and a known object placed in
  front of the robot in-sim, assert `ros2 topic echo --once /oakd/nn/spatial_detections` returns a
  labeled detection with a finite 3D pose. Save the raw message to `verify/artifacts/`.
- **Phase 3 — lidar/SLAM regression guard.** Re-run `./verify/check_topics.sh` (base contract must
  still pass — `/scan` unchanged: frame `rplidar_link`, filter at `SCAN_MIN_RANGE`). Build a map by
  scripted teleop, save it to `maps/`, and confirm the saved `.pgm/.yaml` exist and are non-empty.
  Don't attempt RViz/visual nav sign-off — that's mine.

## Unattended operating rules
- **Background sim, single instance.** Keep one headless+rendering sim alive as a background task;
  every check talks to it. Killing/respawning is fine, but never two at once.
- **Hard timeouts.** Wrap every `ros2`/sim command in `timeout` so a hang doesn't burn the night.
- **Context hygiene** (this is what derails overnight runs): suppress verbose stdout (`>/dev/null`,
  `--once`, short `hz` windows), don't cat large files, and **after each phase write a 3–5 line
  summary to `STATUS.md` and `/compact`** so a fresh context can resume from `STATUS.md` alone.
- **Resume protocol.** On (re)start, read `STATUS.md` and continue from its "Next action". The
  `SessionStart` hook prints it for you.
- **Stop conditions (leave it for me):** any phase gate fails twice; the baseline goes red; a step
  needs the GUI; you're unsure whether a change is destructive; or you'd need to touch the
  real-robot config / a second Isaac instance to proceed.

## Done = Phases 1–3 green on the branch
End state: branch `feat/oakd-phases-1-3` with per-step commits, all three scripted gates green,
`verify/artifacts/` populated for my review, `STATUS.md` current, `README.md`/`COMMANDS.md` updated
with new `isaac-*` aliases. Then STOP and summarize in `STATUS.md`. I'll do the visual sign-off and
Phase 4 (HMI extension) when I'm back.
