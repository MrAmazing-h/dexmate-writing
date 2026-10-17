"""Evaluate physical rollout independently of the renderer."""

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from . import contact_writing as c

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
out = a.output
m, d = c.load(out)
z = np.load(out / "rollout.npz")
ref = np.load(out / "reference.npz")
q = z["qpos"]
s = z["stats"]
down = z["pen_down"]
contact = s[:, 3] > 0.005
xy = np.linalg.norm(s[:, :2] - z["target"][:, :2], axis=1) * 1000


def summarize(x):
    return (
        dict(mean=float(np.mean(x)), p95=float(np.quantile(x, 0.95)), max=float(np.max(x)))
        if len(x)
        else None
    )


body = m.body("R_hand_base").id
pen = m.body("free_pen").id
sr = m.site("pen_reference_tip").id
rel_R = Rotation.from_quat(m.site_quat[sr][[1, 2, 3, 0]]).as_matrix()
offset = m.site_pos[sr] - rel_R @ m.site_pos[m.site("pen_tip").id]
slip = []
angle = []
recomputed = []
penetrations = {}
ref_penetrations = {}
reference_errors = []
for i, row in enumerate(q):
    d.qpos[:] = row
    mujoco.mj_forward(m, d)
    R = d.xmat[body].reshape(3, 3)
    slip.append(np.linalg.norm(R.T @ (d.xpos[pen] - d.xpos[body]) - offset) * 1000)
    angle.append(
        np.linalg.norm(Rotation.from_matrix(rel_R.T @ R.T @ d.xmat[pen].reshape(3, 3)).as_rotvec())
        * 180
        / np.pi
    )
    recomputed.append(np.linalg.norm(d.site("pen_tip").xpos - s[i, :3]))
    for ct in d.contact:
        if ct.dist < 0:
            pair = " / ".join(
                m.body(m.geom_bodyid[g]).name + ":" + m.geom(g).name for g in [ct.geom1, ct.geom2]
            )
            penetrations[pair] = max(penetrations.get(pair, 0.0), -float(ct.dist) * 1000)
    ri = int(z["reference_index"][i]) if "reference_index" in z else i
    d.qpos[:] = ref["qpos"][ri, : m.nq]
    mujoco.mj_forward(m, d)
    reference_errors.append(
        np.linalg.norm(d.site("pen_reference_tip").xpos - ref["target"][ri]) * 1000
    )
    for ct in d.contact:
        names = [m.geom(g).name for g in [ct.geom1, ct.geom2]]
        bodies = [m.body(m.geom_bodyid[g]).name for g in [ct.geom1, ct.geom2]]
        intended = (
            ("pen_nib" in names and ("paper" in names or "table_top" in names))
            or ("free_pen" in bodies and any(b.startswith("R_") for b in bodies))
            or ("table_top" in names and any(b.startswith("L_") for b in bodies))
        )
        if ct.dist < 0 and not intended:
            pair = " / ".join(b + ":" + n for b, n in zip(bodies, names))
            ref_penetrations[pair] = max(ref_penetrations.get(pair, 0.0), -float(ct.dist) * 1000)
arms = [
    m.jnt_qposadr[m.joint(side + "_arm_j" + str(k)).id] for side in ["L", "R"] for k in range(1, 8)
]
stroke_results = []
ink_samples = z["ink_contact_samples"] if "ink_contact_samples" in z else np.empty((0, 6))
ink_errors = []
for sid in np.unique(z["stroke_id"][down]):
    mask = down & (z["stroke_id"] == sid)
    path = ref["target"][ref["pen_down"] & (ref["stroke_id"] == sid), :2]
    length = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    arc = np.linspace(0, length[-1], max(2, int(length[-1] / 0.00025) + 1))
    dense = np.column_stack([np.interp(arc, length, path[:, k]) for k in range(2)])
    measured = ink_samples[ink_samples[:, 1] == sid, 2:4]
    dist = (
        cKDTree(measured).query(dense)[0] * 1000 if len(measured) else np.full(len(dense), np.inf)
    )
    ink_errors.extend(dist)
    stroke_results.append(
        {
            "stroke": int(sid),
            "contact_coverage": float(np.mean(contact[mask])),
            "xy_error_mm": summarize(xy[mask]),
            "ink_path_within_1mm_fraction": float(np.mean(dist <= 1)),
            "ink_path_max_distance_mm": float(np.max(dist)),
        }
    )
up_ink = ink_samples[ink_samples[:, 1] < 0, 2:4]
up_distance = (
    cKDTree(ref["target"][ref["pen_down"], :2]).query(up_ink)[0] * 1000
    if len(up_ink)
    else np.array([])
)
metrics = {
    "text": str(ref["text"]),
    "duration_s": len(q) / int(z["fps"]),
    "physics_dt_s": m.opt.timestep,
    "record_fps": int(z["fps"]),
    "model": "DexMate Vega-1P with F5D6",
    "base_fixed": True,
    "robot_gravity_compensation": True,
    "gravity_m_s2": m.opt.gravity.tolist(),
    "holding_mode": json.loads((out / "controller.json").read_text()).get(
        "holding_mode", "finger_friction_only"
    ),
    "pen_weld_constraints": int(np.sum(m.eq_type == int(mujoco.mjtEq.mjEQ_WELD))),
    "inherited_body_pair_exclusions": int(m.nexclude),
    "contact_collision_geoms": int(np.count_nonzero(m.geom_contype)),
    "pen_down_contact_coverage": float(np.mean(contact[down])),
    "pen_down_xy_tracking_error_mm": summarize(xy[down]),
    "reference_ik_error_mm": summarize(np.array(reference_errors)),
    "ink_path_within_1mm_fraction": float(np.mean(np.array(ink_errors) <= 1)),
    "ink_path_distance_mm": summarize(np.array(ink_errors)),
    "ink_path_metric": "Reference strokes resampled at approximately 0.25 mm arc spacing; nearest same-stroke positive-force physical nib-paper contact point in XY.",
    "maximum_recorded_nib_contact_force_N": float(ink_samples[:, 5].max())
    if len(ink_samples)
    else 0.0,
    "pen_up_contact_distance_to_intended_strokes_mm": summarize(up_distance),
    "pen_normal_force_N_during_writing": summarize(s[down, 3]),
    "left_support_contact_coverage_after_1s": float(np.mean(s[100:, 4] > 0.005)),
    "left_support_normal_force_N_after_1s": summarize(s[100:, 4]),
    "pen_attachment_translation_error_mm": summarize(np.array(slip)),
    "pen_attachment_rotation_error_deg": summarize(np.array(angle)),
    "arm_joint_velocity_rad_s": summarize(
        np.abs(np.diff(q[:, arms], axis=0)).ravel() * int(z["fps"])
    ),
    "recorded_tip_fk_consistency_max_m": max(recomputed),
    "rollout_penetration_by_pair_mm": penetrations,
    "reference_unintended_penetration_by_pair_mm": ref_penetrations,
    "per_stroke": stroke_results,
    "ink_rule": "All measured physical nib-paper contact positions with normal force exceeding 0.005 N, including pen-up contacts; surface projection for rendering.",
    "limitations": [
        "Manually loaded pen clamp is modeled as a fixed tool attachment (or a compliant weld in earlier candidates); this is not verified friction-only finger grasping.",
        "Robot base is fixed and robot links use gravity compensation.",
        "Inherited self-collision exclusions remain; enabled contact pairs are audited.",
        "No real-robot validation; actuator and friction parameters are simulation settings.",
    ],
}
(out / "metrics.json").write_text(json.dumps(metrics, indent=2))
print(
    json.dumps(
        {
            k: v
            for k, v in metrics.items()
            if k not in ("per_stroke", "rollout_penetration_by_pair_mm")
        },
        indent=2,
    )
)
