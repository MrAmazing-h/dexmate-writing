# Changelog

## 0.3.0 — WriteUrRobo

- Renamed the showcase to cover both DexMate and G1; retained the original package/import names and DexMate CLIs.
- Added a separately configured fixed-base G1 + Inspire DFQ writing station, a pinned and checksum-verified public mesh downloader, seven-joint writing IK, position-servo execution and actual contact recording.
- Added a recorded-state renderer with whole-robot, hand close-up and measured-contact XY views. Kinematic reference paths are not used as visible ink.
- Added explicit writing-tracking checks and a packaging gate that rejects failed/truncated records, plus portable record replay with a verified scene hash.
- Preserved all old DexMate control code, original videos and earlier release assets. G1's fixed fixture and compact workspace are not claims of friction grasping, balancing or hardware readiness.

## 0.2.0 — 2026-10-16

- Add printable-ASCII layout and an isolated contact-writing CLI, reusing the evaluated October 4 controller.
- Publish a checksummed calibrated station, recorded reference/rollout data and two full contact-writing videos.
- Refresh the README with a 4× preview, overhead ink images, grip/support views, and measured metrics.
- State the rigid pen attachment, fixed base, gravity compensation, collision exclusions and unresolved force peaks explicitly.
- Preserve the v0.1.0 Chinese kinematic CLI, data and release.

## 0.1.0 — 2026-10-01

Initial public release of the previously demonstrated kinematic workflow, with portable commands, numerical diagnostics, example trajectories, provenance, and tests.
