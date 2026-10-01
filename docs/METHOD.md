# Method

## From characters to a timed path

The eight unique character JSON files are unmodified Hanzi Writer Data medians. The character `天` occurs twice. Each stroke is parameterized by cumulative polyline length and resampled with PCHIP. Glyph coordinates map to a fixed plane: world +X points toward the top of the written character, and world -Y points rightward along the line. The glyph scale is 43 mm and character spacing is 45 mm.

A quintic blend `s(t) = 10t^3 - 15t^4 + 6t^5` eases traversal within each segment. The trajectory contains an initial hold, raised transfers, 13 mm pen lifts, pen-down strokes, retraction, and a nine-second closing camera sequence. The four-second camera transition ends with a five-second overhead hold.

## Fixed pen and IK

The pen is attached rigidly to `R_hand_base`. Its local axis is the normalized vector `[0.75, -0.55, 0.36]`; the grasp point is `[-0.115, -0.027, -0.063]` m. The tip is 40 mm below the grasp point along that axis. Finger joints are preset to a forward grip and coupled joints follow their model mimic relations.

Seven right-arm joints are optimized. The task residual concatenates 3D tip position error and `0.15 * (desired_pen_axis - current_pen_axis)`. Roll about the pen axis remains free. MuJoCo supplies the point and angular Jacobians. Damped least squares uses `dq = J.T @ solve(J @ J.T + 1e-6 I, error)`, caps the maximum step at 0.12 rad, and clips arm joints inside their position limits. The initial pose uses bounded least squares with deterministic random restarts (seed 1); subsequent frames warm-start from the previous solution.

The mobile base, torso, left arm and preset finger joints remain fixed during writing. Nonzero torso angles are intentional; actual FK, not the angle signs alone, defines torso orientation.

## Rendering and evaluation

The animation assigns qpos and calls `mj_forward`; it does not integrate dynamics. Paper and pen geometry are visual. During pen-down intervals, adjacent actual FK tip positions create ink segments only within the same stroke. Their Z coordinate is projected onto the paper; vertical tracking error is therefore measured separately. The phrase is not preprinted on the sheet.

Validation recomputes tip FK from saved qpos, compares it with both target and stored tip arrays, checks joint limits, verifies all strokes/characters and positive paper margins, and requires maximum pen-down position error below 1 mm. Joint velocity is reported, not claimed to meet hardware limits. This is a visual feasibility check, not collision, force or stability certification.
