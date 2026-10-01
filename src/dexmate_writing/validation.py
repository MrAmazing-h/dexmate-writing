"""Independent FK evaluation of saved writing trajectories."""

import json
from pathlib import Path

import mujoco
import numpy as np

from .model_contract import validate_model


def validate(run: Path):
    model = mujoco.MjModel.from_xml_path(str(run / "model/scene.xml"))
    validate_model(model)
    data = mujoco.MjData(model)
    with np.load(run / "writing_reference.npz", allow_pickle=False) as z:
        q = z["qpos"]
        if q.ndim != 2 or q.shape[1] != model.nq or not np.isfinite(q).all():
            raise ValueError("Invalid qpos array")
        actual = []
        for row in q:
            data.qpos[:] = row
            mujoco.mj_forward(model, data)
            actual.append(data.site_xpos[model.site("pen_tip").id].copy())
        actual = np.asarray(actual)
        down = z["pen_down"].astype(bool)
        error = np.linalg.norm(actual - z["pen_tip_target"], axis=1) * 1000
        violations = []
        for j in range(model.njnt):
            if model.jnt_limited[j]:
                values = q[:, model.jnt_qposadr[j]]
                if np.any(values < model.jnt_range[j, 0] - 1e-6) or np.any(
                    values > model.jnt_range[j, 1] + 1e-6
                ):
                    violations.append(model.joint(j).name)
        arm = [model.joint(f"R_arm_j{k}").qposadr[0] for k in range(1, 8)]
        report = {
            "frames": len(q),
            "fps": float(z["fps"]),
            "finite_qpos": True,
            "tip_error_mean_mm": float(error.mean()),
            "tip_error_max_mm": float(error.max()),
            "pen_down_tip_error_mean_mm": float(error[down].mean()),
            "pen_down_tip_error_max_mm": float(error[down].max()),
            "saved_tip_vs_recomputed_fk_max_m": float(np.abs(actual - z["pen_tip_actual"]).max()),
            "joint_position_limit_violations": violations,
            "all_53_strokes_present": set(z["stroke_id"][down]) == set(range(53)),
            "all_9_characters_present": set(z["character_index"][down]) == set(range(9)),
            "right_arm_max_velocity_rad_s": float(
                np.abs(np.diff(q[:, arm], axis=0)).max() * float(z["fps"])
            ),
            "writing_surface_height_error_max_mm": float(
                np.abs(actual[down, 2] - 0.9033).max() * 1000
            ),
            "paper_margin_min_m": float(
                np.column_stack(
                    [
                        actual[down, 0] - 0.28,
                        0.46 - actual[down, 0],
                        actual[down, 1] + 0.355,
                        0.145 - actual[down, 1],
                    ]
                ).min()
            ),
            "scope": "Kinematic FK consistency and a 1 mm pen-down visual error gate; not grasp or force validation.",
        }
    report["passed"] = bool(
        not violations
        and report["all_53_strokes_present"]
        and report["all_9_characters_present"]
        and report["pen_down_tip_error_max_mm"] < 1
        and report["saved_tip_vs_recomputed_fk_max_m"] < 1e-9
        and report["paper_margin_min_m"] > 0
    )
    (run / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise RuntimeError("Writing validation failed; see validation.json")
    return report
