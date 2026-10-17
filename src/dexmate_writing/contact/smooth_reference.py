"""Smooth arm joints, then restore tip targets with local collision-aware IK."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from . import contact_writing as c

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
m, d = c.load(a.output)
z = np.load(a.output / "reference.npz")
data = {k: z[k] for k in z.files}
q = data["qpos"][:, : m.nq].copy()
js = [m.joint("R_arm_j" + str(i)).id for i in range(1, 8)]
qa = m.jnt_qposadr[js]
lo = m.jnt_range[js, 0] + 0.001
hi = m.jnt_range[js, 1] - 0.001
q[:, qa] = savgol_filter(q[:, qa], 51, 3, axis=0)
gids = [
    g
    for g in range(m.ngeom)
    if m.geom_contype[g]
    and m.body(m.geom_bodyid[g]).name.startswith(
        ("R_arm_", "R_hand_", "R_th_", "R_ff_", "R_mf_", "R_rf_", "R_lf_")
    )
]
table = m.geom("table_top").id
errors = []
for i, row in enumerate(q):
    prior = row[qa].copy()
    target = data["target"][i]

    def fun(x):
        d.qpos[:] = row
        d.qpos[qa] = x
        mujoco.mj_forward(m, d)
        ds = np.array([mujoco.mj_geomDistance(m, d, g, table, 0.02, None) for g in gids])
        return np.r_[
            d.site("pen_reference_tip").xpos - target,
            0.02 * (x - prior),
            3 * np.minimum(ds - 0.003, 0),
        ]

    sol = least_squares(fun, np.clip(prior, lo, hi), bounds=(lo, hi), max_nfev=30, ftol=1e-7)
    row[qa] = sol.x
    fun(sol.x)
    errors.append(np.linalg.norm(d.site("pen_reference_tip").xpos - target))
    if i % 500 == 0:
        print("REFINE", i, len(q), errors[-1], flush=True)
data["qpos"] = q
np.savez_compressed(a.output / "reference.npz", **data)
report = {
    "max_tip_error_mm": max(errors) * 1000,
    "p95_tip_error_mm": float(np.quantile(errors, 0.95)) * 1000,
    "max_arm_speed_rad_s": float(np.max(abs(np.diff(q[:, qa], axis=0)))) * 100,
    "max_arm_acceleration_rad_s2": float(np.max(abs(np.diff(q[:, qa], n=2, axis=0)))) * 10000,
}
(a.output / "reference_smoothing.json").write_text(json.dumps(report, indent=2))
print(report, flush=True)
