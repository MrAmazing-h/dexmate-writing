# G1 + Inspire writing station

This is a **fixed-base contact-writing experiment**, not a balancing policy. The original DexMate writing commands and records are preserved. No SONIC weights, RL training, private files, or robot connection are required.

## Reproduce

```bash
pip install -e '.[dev]'
g1-fetch-writing-model --output external/g1
g1-contact-write --model-dir external/g1 --text "Hello, Lab." --output outputs/g1_hello
MUJOCO_GL=egl g1-render-writing --output outputs/g1_hello
```

Use a new output directory: the writer refuses to overwrite an existing run. Planning and physics are CPU work; rendering uses OpenGL (EGL with a supported GPU, or `MUJOCO_GL=osmesa` with an installed OSMesa backend). On a cluster, submit these commands to a compute allocation, not the login node. The `--smoke` option truncates to eight simulated seconds and is **not** a completed-text demonstration.

## Model and station

- G1 29-DOF revision 1.0 + Inspire DFQ hands, derived from Unitree's official robot description at commit `9926cc2f179ae3b86f4f74087bd32ef0c8b6fd90`. The manifest pins every downloaded mesh URL and SHA-256. A changed cached file is rejected, not overwritten.
- The packaged MuJoCo conversion includes the hand mimic equalities. The writing scene removes the floating-base joint, anchors the pelvis, and applies gravity compensation to robot links. The passive nib still experiences gravity.
- The right wrist carries a **rigid pen fixture**, not a simulated friction grasp. The fingers are held at their reference configuration; no claim of dexterous pen acquisition is made.
- The nib has a 6 mm passive slide, 500 N/m spring and 0.5 N·s/m damping. Paper top is world Z = 0.750 m. These are experimental simulation settings, not measured hardware calibration.
- The model's actuator effort bounds remain in effect. Right-arm position servos use Kp = 250 and Kv = 16; hands use 8 / 0.4; other driven hinges use 120 / 8. There is no learned controller.

## Reference and execution

The existing Hershey `futural` ASCII layout is translated into the G1 station, with at most 12 mm capital height and a 120 × 50 mm writing area. Larger layouts are rejected; use shorter text or explicit line breaks. This compact workspace differs from the DexMate station. Page-right is world −Y, page-down is −X, and world +Z points upward. A seven-joint right-arm damped-Jacobian IK tracks tip position and a constant wrist orientation, respecting joint limits. Initialization uses continuation from the paper centre to the first glyph. Nominal writing speed is 18 mm/s. Pen-up travel and lowering/lifting are explicit reference segments.

The 100 Hz joint reference is executed through position actuators with ten 1 ms physics steps per sample. A slow planar Cartesian-error integral correction is mapped back to the arm and bounded to ±0.06 rad per joint. The vertical error is not integrated against the paper: the unloaded nib target deliberately preloads the spring. This is **position-error feedback with a compliant nib**, not the DexMate controller's force-feedback law. Normal forces are measured and reported, not directly regulated by a force controller.

The simulation starts at the first IK hover pose, at rest. It does not verify an approach from a home posture. It does not guarantee collision-free motion: enabled negative contact distances are retained in `metrics.json`, including inherited hand self-contacts. No collision masks are disabled to make the writing result look successful.

## What is recorded and shown

| File | Meaning |
| --- | --- |
| `scene.xml` | Actual generated station; relative mesh path requires the fetched mesh directory to remain present |
| `layout.json`, `layout_preview.png` | Planned font layout; `g1_translation_m` maps the base layout into this station |
| `reference.npz` | Planned qpos, tip target, pen-down flag, stroke ID, 100 Hz |
| `rollout.npz` | Simulated qpos/qvel, tip position, contact force, contact ink, time, joint names and qpos indices |
| `metrics.json` | Time-aligned tip error, measured-contact fraction, force peak and negative contact distances |
| `writing_checks.json` | Explicit tracking/contact checks, not force, collision or hardware safety acceptance |
| `videos/g1_contact_writing.mp4` | Recorded physics states at original speed, hand close-up and actual XY contact trace |

Ink uses measured positive-force nib–paper contacts, including unintended contacts during nominal pen-up intervals. The renderer does not paint the planned glyph. It connects only nearby contacts of the same stroke with no long gap. The XY panel is a labelled projection of actual contact positions, not an ideal font or a camera image. Contact dots are retained even when a segment is not connected.

To replay a downloaded record after moving its directory, use `g1-render-writing --output PATH_TO_RECORD --model-dir external/g1`. The original scene hash is checked before overriding only the mesh search path in memory. Fetching meshes verifies their pinned hashes; no private machine paths are required.

Position is in metres, revolute joints in radians, time in seconds, force in newtons. `qpos` includes the passive nib slide; do not send this vector blindly to hardware. `time_s` describes each recorded post-step state; its first sample is 0.01 s. The reference sample at index i is the command applied over [i/100, (i+1)/100].

## Limits

The complete `Hello, Lab.` run was repeated using the built wheel rather than the source-tree import. All 6,887 recorded qpos/qvel samples, tip positions, force samples, ink, targets and timestamps were identical in that test. [Reproduction report and tested dependency versions](../results/g1/portable_reproduction.json). This is one deterministic reproduction, not a guarantee across every MuJoCo/CPU/backend version.

These examples demonstrate contact writing under stated simulation assumptions. They do **not** establish unsupported standing balance, self-collision clearance, safe pen pressure, grasp robustness, arbitrary-text success, or real-robot deployment. The two robot controllers, station dimensions and demonstrations differ; their numbers are not a controlled comparison. The overview artwork is decorative, while demonstration ink is measured.
