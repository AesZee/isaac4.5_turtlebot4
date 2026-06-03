# STATUS — unattended Phases 1–3

Resume protocol: read this top-to-bottom, then continue from **Next action**. Update the phase
table + Next action + Log after every completed or failed step. Keep entries short (context hygiene).

## Branch
- work branch: _(set on first run, e.g. feat/oakd-phases-1-3)_
- last green commit: _(sha)_

## Phases
| phase | state | last gate | notes |
|-------|-------|-----------|-------|
| baseline (verified contract) | not started | — | `./verify/check_topics.sh` must PASS before Phase 1 |
| 1 — OAK-D RGB + depth + points | not started | — | extend gate with `/oakd/*`; save sample frames to verify/artifacts/ |
| 2 — detection + spatial 3D | not started | — | assert `/oakd/nn/spatial_detections` with object in view |
| 3 — lidar/SLAM regression guard | not started | — | base contract still green; build+save a map |
| 4 — HMI extension | **DEFERRED (manual/GUI)** | — | do NOT attempt unattended |

## Next action
Reconcile `verify/check_topics.sh` with the real `isaac-ros` env, bring up the headless+rendering
sim as a background task, run the gate to get a green baseline, commit, then start Phase 1.

## Stop conditions (leave for human)
A gate fails twice · baseline goes red · a step needs the GUI · a change might be destructive ·
proceeding would require touching the real-robot config or a second Isaac instance.

## Log
- _(timestamp)_ initialized.
