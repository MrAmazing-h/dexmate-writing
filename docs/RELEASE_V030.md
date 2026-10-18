# WriteUrRobo v0.3.0 — DexMate × G1

A shared writing showcase for DexMate Vega-1P and Unitree G1 with Inspire DFQ hands. The repository was previously named `dexmate-writing`; the Python package/import name and old commands are retained for compatibility.

## Watch and reproduce

- `writeurrobo_showcase.mp4`: both recorded simulations at original speed, with explicitly labelled final-frame holds. Different texts/stations/controllers: not a synchronized benchmark.
- `g1_contact_writing.mp4`: the complete G1 run, including whole-body view, hand close-up and a projection of actual contact ink.
- `writeurrobo_g1_contact_record_v030.zip`: actual G1 states, planned reference, scene, joint ordering, timestamps, metrics, video, English README and checksums. Public meshes are fetched separately with pinned hashes.
- Python wheel/source distribution: installable code, model template, asset manifest, tests and documentation.

G1 wrote `Hello, Lab.` over 68.87 simulated seconds: mean pen-down XY error 0.955 mm, P95 1.673 mm, measured contact fraction 95.08%, peak recorded nib normal force 0.396 N. These are one-example tracking measurements, not general handwriting performance or a safety certificate.

## Scope and limitations

Both stations have fixed bases, robot-link gravity compensation and rigidly attached pens with passive compliant nibs. G1 uses position servos with bounded planar-error integral correction; DexMate retains its prior position/force-feedback controller. No new RL, autonomous grasp or balancing policy is introduced.

G1 enabled-contact overlaps remain (including a 5.20 mm thumb/fixture mesh contact), as do nominal pen-up contact samples. They are retained in the audit and ink, not concealed. Original DexMate force-peak and collision-exclusion limitations also remain. No hardware-control code is provided and no real-robot safety has been demonstrated.

## Compatibility

The original DexMate controller, kinematic pipeline, Chinese phrase assets, v0.1.0/v0.2.0 releases and their videos are preserved. New G1 modules and commands are separate. See the repository README and `docs/G1_WRITING.md` for complete commands, experimental settings, coordinate conventions and limits.
