# Reproducibility

Use the versioned robot conversion from `dexmate-fetch-model`. Its archive SHA-256 is embedded in the downloader; per-file hashes, upstream attribution and conversion provenance are included in the ZIP. Preserve `robot.xml`, `robot.urdf` and the relative `meshes/` directory together.

The source release includes the exact stroke subset and the recorded trajectory. The full generation requires only the core package dependencies. There are no learned weights. Run build, solve, validate and render as shown in the README. Use a new output directory for each experiment. Model contract validation checks joint names and qpos addresses, not only the vector length.

The reference configuration uses 60 Hz qpos and 30 Hz video, seed 1 for initial IK search, and the fixed constants in `handwriting.py`. The published runtime uses MuJoCo 3.10.0, NumPy 2.3.5, SciPy 1.17.1 and Python 3.11. Floating-point results can vary across BLAS implementations; compare errors and qpos rather than requiring video bytes to match. The public renderer uses portable English labels and initializes FK before camera fitting; the underlying stroke and IK equations are preserved.

`results/recorded_evaluation.json` is the original demonstration record. `results/release_validation.json` reports fresh generation with the packaged code. The original full video is distributed as a release asset. `examples/writing_reference.npz` contains `qpos[T,55]`, scalar `fps`, `pen_tip_target[T,3]`, `pen_tip_actual[T,3]`, `pen_down[T]`, `stroke_id[T]`, `character_index[T]`, joint names/addresses and camera phase markers. Position units are metres, hinge angles radians, root quaternion order is wxyz.

CI tests code/data invariants and builds a wheel on Python 3.11 and 3.12. Full rendering and model-dependent reproduction are release checks, not a claim that every CI runner has OpenGL. macOS and Windows have not been validated.
