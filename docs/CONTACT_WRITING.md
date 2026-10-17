# Contact-writing method and reproduction

## Pipeline

1. `text_layout.py` loads Hershey `futural` strokes, wraps words or splits long words, preserves explicit newlines and fits the page. Page-right is world −Y; page-down is world −X. The calibrated bounds are X=[0.345,0.415] m and Y=[−0.300,−0.020] m. These bounds do not prove reachability of arbitrary tasks.
2. `contact_writing.plan` adds quintic travel, pen lifts, per-stroke arc-length sampling and retraction. The right-arm reference is solved with bounded least squares, an orientation preference and hand/arm-to-table clearance penalties. A calibrated posture supplies the fingers and left support hand.
3. `smooth_reference` applies a 51-sample cubic Savitzky–Golay filter to the right arm, followed by local tip/clearance refinement. This regularizes the reference; it does not repair arbitrary collisions.
4. `contact_writing.simulate` executes 100 Hz actuator targets with 2 kHz MuJoCo physics using the downloaded **final calibrated scene**. The historical scene-building helper is not the reproducible station-construction recipe: use the checksummed asset for the evaluated setup.
5. `evaluate_contact` reconstructs FK, compares the logged tip to the reference and measures ink coverage, contact and enabled-pair penetration. `render_contact` renders recorded states; it does not run new physics or draw the planned paths.

## Feedback actually changes the command

At each 100 Hz interval, the XY bias integrates reference-tip minus measured-tip error with gain 0.03 and is clipped to ±4 mm. The vertical offset integrates measured normal force minus the 0.8 N target with gain 3e−6 m/N per control interval, clipped to [−2, +0.5] mm. A 0.5 mm preload ramps with alpha=0.08 during writing. These are discrete controller gains, not time-normalized continuous-time coefficients.

A damped Jacobian solve maps the Cartesian correction into right-arm offsets: dq = Jᵀ(JJᵀ + 1e−5 I)⁻¹ e, with orientation rows scaled by 0.15; the resulting joint offset is bounded by 0.1 rad. Position-servo targets combine the reference, this feedback correction and velocity feedforward. The left arm also adjusts a small support preload using its measured table-contact force.

A stroke waits until normal force exceeds 0.05 N and XY error is below 1 mm; contact dropout can pause the reference phase again. A prolonged hold fails explicitly. The rollout stores `reference_index`, so time-aligned evaluation follows the reference phase rather than incorrectly comparing equal array indices after a pause.

This is a custom implementation of conventional feedback control. It is not a learned grasp policy or a new general dexterous-hand algorithm. The measured contact forces come from the simulator, not from verified real DexMate tactile hardware.

## Physics and attachment

The base is fixed, robot links use gravity compensation, and the pen is rigidly attached to the hand with a passive compliant nib. Normal pen/table and left-hand/table interactions are simulated. The hand does not need friction to keep hold of the pen. The old candidates included free-pen or compliant-weld trials; they are not the released two-line controller. No continuous table-pickup-to-writing demo is claimed.

Self-collision exclusions inherited from the robot conversion remain. Reported maximum penetration is across **enabled contacts at every physics substep**, not all visual mesh pairs. The 21.270 N two-line peak nib force is an unresolved transient despite the nominal 0.8 N target. Do not describe the current controller as force-safe.

## Evaluation

Acceptance checks finite recorded qpos/stats/targets, >=99% reference coverage within 1 mm of same-stroke measured ink, mean pen-down XY tracking error <1 mm, >=98% pen-down contact coverage, no enabled contact deeper than 1 mm, and no reference unintended-contact penetration above 1 mm. There is no force-safety acceptance limit, hold-force validation, or real-robot test.

Ink uses positive-force (>0.005 N) nib–paper contacts, including pen-up contacts. The renderer combines substep samples into force-weighted per-control-interval positions and projects them onto the paper for visibility. Short spatially continuous samples are connected within each stroke. Geometric coverage and time-aligned tip error answer different questions.

## Reproduce or inspect

The two-line recording used `--max-cap-mm 25`, text `DexMate 2026!\nHello, Lab.`, the released station and controller JSON. The single-line `Yuxiong Lab` recording used earlier hand-authored glyph paths; it is a comparison showcase, not a bit-identical output of the new Hershey text command.

To independently evaluate the recorded run, unpack `contact_recorded_demo.zip`, link its `model` directory to `vega_1p_contact_station/model`, and run:

```bash
python -m dexmate_writing.contact.evaluate_contact --output contact_recorded_demo
MUJOCO_GL=egl python -m dexmate_writing.contact.render_contact --output contact_recorded_demo
```

The release archive includes its own controller and layout. Keep source recordings intact and evaluate a separate copy. To regenerate from text, use the README command. Numerical simulation can vary by platform; the release does not promise bit-identical trajectories across MuJoCo versions or CPUs.

The v0.2.0 packaging update reuses the October 2–4 physical demonstrations. Code changes isolate package imports, add explicit asset download and path handling, preserve prior results, and expose the CLI. See `results/contact/release_validation.json` for exactly which checks were rerun; archived full videos are not silently presented as new rollouts.
