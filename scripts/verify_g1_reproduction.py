"""Compare a wheel-installed reproduction with its recorded source-tree run."""

import argparse
import importlib.metadata
import json
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("baseline", type=Path)
p.add_argument("reproduction", type=Path)
p.add_argument("--report", type=Path, required=True)
a = p.parse_args()
errors = {}
with np.load(a.baseline / "rollout.npz") as old, np.load(a.reproduction / "rollout.npz") as new:
    for key in ("qpos", "qvel", "tip", "force", "ink", "target", "time_s"):
        assert old[key].shape == new[key].shape, key
        errors[key] = float(np.max(np.abs(old[key] - new[key])))
        assert np.allclose(old[key], new[key], atol=1e-5, rtol=1e-5), (key, errors[key])
    for key in ("contact", "pen_down", "stroke_id", "joint_names", "qpos_addresses"):
        assert np.array_equal(old[key], new[key]), key
report = {
    "status": "PASS",
    "scope": "Complete source-tree vs installed-wheel physical rollout; no hardware or safety acceptance",
    "max_absolute_differences": errors,
    "versions": {
        name: importlib.metadata.version(name)
        for name in ("numpy", "scipy", "mujoco", "Hershey-Fonts", "imageio", "Pillow")
    },
}
a.report.parent.mkdir(parents=True, exist_ok=True)
a.report.write_text(json.dumps(report, indent=2))
print(json.dumps(report))
