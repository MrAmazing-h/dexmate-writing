"""G1 + Inspire writing with a rigid pen fixture and compliant contact nib."""

import argparse
import hashlib
import json
import os
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ..contact.text_layout import layout

ARM = [
    f"right_{n}_joint"
    for n in (
        "shoulder_pitch",
        "shoulder_roll",
        "shoulder_yaw",
        "elbow",
        "wrist_roll",
        "wrist_pitch",
        "wrist_yaw",
    )
]
TABLE_Z = 0.75


def make_scene(model_dir, out):
    tree = ET.ElementTree(
        ET.fromstring(files(__package__).joinpath("data/g1_inspire.xml").read_text())
    )
    root = tree.getroot()
    root.set("model", "G1_Inspire_fixed_writing_station")
    root.find("compiler").set("meshdir", os.path.relpath(model_dir / "meshes", out))
    root.find("option").attrib.update(
        timestep="0.001", gravity="0 0 -9.81", integrator="implicitfast"
    )
    root.find(".//freejoint/..").remove(root.find(".//freejoint"))
    for body in root.findall(".//body"):
        body.set("gravcomp", "1")
    base = root.find("worldbody/body")
    base.set("pos", "0 0 0.793")
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_visual = visual.find("global")
    if global_visual is None:
        global_visual = ET.SubElement(visual, "global")
    global_visual.set("offwidth", "1280")
    global_visual.set("offheight", "720")
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ET.SubElement(visual, "headlight")
    headlight.attrib.update(ambient=".3 .3 .3", diffuse=".5 .5 .5")
    world = root.find("worldbody")
    for name, pos, size, rgba in (
        ("desk", ".44 -.12 .716", ".27 .38 .03", ".36 .23 .14 1"),
        ("paper", ".35 -.20 .748", ".085 .14 .002", ".97 .97 .93 1"),
    ):
        ET.SubElement(
            world,
            "geom",
            name=name,
            type="box",
            pos=pos,
            size=size,
            rgba=rgba,
            friction=".6 .01 .001",
            solref=".005 1",
        )
    wrist = root.find('.//body[@name="right_wrist_yaw_link"]')
    pen = ET.SubElement(wrist, "body", name="writing_fixture", pos=".19 .055 0", gravcomp="1")
    # The pen is a fixture, not a friction-only grasp. Orientation calibrated below.
    ET.SubElement(
        pen,
        "geom",
        name="pen_barrel",
        type="capsule",
        fromto="0 0 -.065 0 0 .065",
        size=".004",
        mass=".012",
        rgba=".08 .25 .65 1",
    )
    nib = ET.SubElement(pen, "body", name="compliant_nib", pos="0 0 -.073", gravcomp="0")
    ET.SubElement(
        nib,
        "joint",
        name="nib_slide",
        type="slide",
        axis="0 0 1",
        range="0 .006",
        stiffness="500",
        damping=".5",
        springref="0",
        armature=".001",
    )
    ET.SubElement(
        nib,
        "geom",
        name="nib",
        type="sphere",
        size=".002",
        mass=".002",
        rgba=".1 .1 .1 1",
        solref=".004 1",
        friction=".5 .01 .001",
    )
    ET.SubElement(nib, "site", name="tip", size=".001", rgba="0 0 0 0")
    root.find("actuator").clear()
    # Preserve model effort limits. All actuated independent hinges hold their targets.
    dependent = {e.get("joint1") for e in root.findall("./equality/joint")}
    for joint in root.findall(".//worldbody//joint"):
        name = joint.get("name")
        if not name or name == "nib_slide" or name in dependent:
            continue
        limit = joint.get("actuatorfrcrange", "-1 1")
        is_arm = name in ARM
        kp, kv = (250, 16) if is_arm else ((8, 0.4) if name.startswith(("R_", "L_")) else (120, 8))
        ET.SubElement(
            root.find("actuator"),
            "position",
            name=name,
            joint=name,
            kp=str(kp),
            kv=str(kv),
            forcerange=limit,
        )
    ET.indent(tree)
    tree.write(out / "scene.xml", encoding="unicode")
    m = mujoco.MjModel.from_xml_path(str(out / "scene.xml"))
    d = mujoco.MjData(m)
    q = m.qpos0.copy()
    for side in ("left", "right"):
        for name, value in (("hip_pitch", -0.312), ("knee", 0.669), ("ankle_pitch", -0.363)):
            q[m.joint(f"{side}_{name}_joint").qposadr[0]] = value
    for name, value in zip(ARM, [0.3, -0.3, 0, 1.2, 0, -0.4, 0]):
        q[m.joint(name).qposadr[0]] = value
    q[m.joint("left_shoulder_roll_joint").qposadr[0]] = 0.2
    for finger in ("index", "middle", "ring", "pinky"):
        for segment in ("proximal", "intermediate"):
            q[m.joint(f"R_{finger}_{segment}_joint").qposadr[0]] = 0.8
    for name, value in (("yaw", 0.8), ("pitch", 0.3)):
        q[m.joint(f"R_thumb_proximal_{name}_joint").qposadr[0]] = value
    q[m.joint("R_thumb_intermediate_joint").qposadr[0]] = 0.48
    q[m.joint("R_thumb_distal_joint").qposadr[0]] = 0.72
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    rotation = d.body("right_wrist_yaw_link").xmat.reshape(3, 3).copy()
    quat = Rotation.from_matrix(rotation.T).as_quat()[[3, 0, 1, 2]]
    pen.set("quat", " ".join(map(str, quat)))
    tree.write(out / "scene.xml", encoding="unicode")
    return mujoco.MjModel.from_xml_path(str(out / "scene.xml")), q, rotation


def solve(m, d, q, target, orientation, iterations=40):
    joints = np.array([m.joint(n).id for n in ARM])
    qa, va = m.jnt_qposadr[joints], m.jnt_dofadr[joints]
    tip, wrist = m.site("tip").id, m.body("right_wrist_yaw_link").id
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(iterations):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        e = target - d.site_xpos[tip]
        rot = Rotation.from_matrix(orientation @ d.xmat[wrist].reshape(3, 3).T).as_rotvec()
        if np.linalg.norm(e) < 1e-6 and np.linalg.norm(rot) < 1e-4:
            break
        mujoco.mj_jacSite(m, d, jp, jr, tip)
        jac = np.vstack((jp[:, va], 0.08 * jr[:, va]))
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6 * np.eye(6), np.r_[e, 0.08 * rot])
        dq *= min(1.0, 0.1 / max(np.max(np.abs(dq)), 1e-10))
        q[qa] = np.clip(q[qa] + dq, m.jnt_range[joints, 0] + 0.002, m.jnt_range[joints, 1] - 0.002)
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    return q, float(np.linalg.norm(target - d.site_xpos[tip]))


def path(strokes, speed=0.018, hz=100):
    pts, down, ids = [], [], []
    current = strokes[0][0] + [0, 0, 0.018]

    def segment(end, writing, sid, v):
        nonlocal current
        n = max(2, int(np.ceil(np.linalg.norm(end - current) / v * hz)))
        u = np.linspace(0, 1, n, endpoint=False)
        blend = u * u * (3 - 2 * u)
        pts.extend(current + blend[:, None] * (end - current))
        down.extend([writing] * n)
        ids.extend([sid] * n)
        current = end.copy()

    for k, stroke in enumerate(strokes):
        segment(stroke[0] + [0, 0, 0.018], False, k, 0.04)
        segment(stroke[0], False, k, 0.01)
        # Arc-length timing within each stroke (no pause at font polyline vertices).
        dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(stroke, axis=0), axis=1))]
        n = max(2, int(np.ceil(dist[-1] / speed * hz)))
        ss = np.linspace(0, dist[-1], n)
        pp = np.stack([np.interp(ss, dist, stroke[:, j]) for j in range(3)], axis=1)
        pts.extend(pp)
        down.extend([True] * n)
        ids.extend([k] * n)
        current = stroke[-1].copy()
        segment(current + [0, 0, 0.018], False, k, 0.01)
    return np.array(pts), np.array(down), np.array(ids)


def run(out, model_dir, text, smoke=False):
    out.mkdir(parents=True, exist_ok=False)
    report = layout(text, out, max_cap_mm=12)
    strokes = [np.asarray(s["points"]) for s in report["strokes"]]
    allp = np.concatenate(strokes)
    if np.ptp(allp[:, 0]) > 0.05 or np.ptp(allp[:, 1]) > 0.12:
        raise ValueError("G1 writing area is 120 x 50 mm; shorten text or add line breaks")
    shift = np.array([0.35, -0.20, TABLE_Z + 0.001]) - (allp.min(0) + allp.max(0)) / 2
    strokes = [p + shift for p in strokes]
    report["g1_translation_m"] = shift.tolist()
    (out / "layout.json").write_text(json.dumps(report, indent=2))
    m, q, orientation = make_scene(model_dir, out)
    d = mujoco.MjData(m)
    targets, down, ids = path(strokes)
    if smoke:
        targets, down, ids = targets[:800], down[:800], ids[:800]
    # Calibrate once at the station centre, then continue to the first glyph.
    # A direct large jump from the seed to a long line's edge can select an
    # infeasible local IK branch. This continuation is independent of the text.
    centre = np.array([0.35, -0.20, targets[0, 2]])
    q, error = solve(m, d, q, centre, orientation, iterations=300)
    if error > 0.002:
        raise RuntimeError(f"Station centre unreachable: {error} m")
    for point in np.linspace(centre, targets[0], 30):
        q, error = solve(m, d, q, point, orientation, iterations=80)
        if error > 0.002:
            raise RuntimeError(f"Initial continuation unreachable: {error} m")
    print("INIT", error, d.site("tip").xpos, flush=True)
    if error > 0.002:
        raise RuntimeError(f"Initial pose unreachable: {error} m")
    qs, errors = [], []
    for i, target in enumerate(targets):
        q, error = solve(m, d, q, target, orientation)
        qs.append(q.copy())
        errors.append(error)
    if max(errors) > 0.002:
        worst = int(np.argmax(errors))
        (out / "reference_failure.json").write_text(
            json.dumps(
                {
                    "frame": worst,
                    "target": targets[worst].tolist(),
                    "error_m": errors[worst],
                    "qpos": qs[worst].tolist(),
                },
                indent=2,
            )
        )
        raise RuntimeError(f"Reference unreachable: {max(errors)} m")
    qs = np.asarray(qs)
    np.savez_compressed(
        out / "reference.npz", qpos=qs, target=targets, pen_down=down, stroke_id=ids, fps=100
    )
    d.qpos[:] = qs[0]
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    driven = m.actuator_trnid[:, 0]
    aq = m.jnt_qposadr[driven]
    armj = np.array([m.joint(n).id for n in ARM])
    av = m.jnt_dofadr[armj]
    armq = m.jnt_qposadr[armj]
    tip = m.site("tip").id
    nib = m.geom("nib").id
    paper = m.geom("paper").id
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    recorded = []
    ink = []
    peak = 0.0
    penetration = {}
    correction = np.zeros(7)
    for i, target in enumerate(targets):
        actual = d.site_xpos[tip].copy()
        mujoco.mj_jacSite(m, d, jp, jr, tip)
        error = target - actual
        # Do not integrate Z against the deliberately preloaded compliant nib.
        planar_error = error.copy()
        planar_error[2] = 0
        # Slow bounded Cartesian integral correction compensates static servo error.
        correction += 0.03 * (
            jp[:, av].T @ np.linalg.solve(jp[:, av] @ jp[:, av].T + 1e-5 * np.eye(3), planar_error)
        )
        correction = np.clip(correction, -0.06, 0.06)
        command = qs[i].copy()
        command[armq] += correction
        command[armq] = np.clip(
            command[armq], m.jnt_range[armj, 0] + 0.001, m.jnt_range[armj, 1] - 0.001
        )
        d.ctrl[:] = command[aq]
        hits = []
        force_sum = 0.0
        for _ in range(10):
            mujoco.mj_step(m, d)
            for cidx, c in enumerate(d.contact):
                pair = {int(c.geom1), int(c.geom2)}
                if pair == {nib, paper}:
                    f = np.zeros(6)
                    mujoco.mj_contactForce(m, d, cidx, f)
                    peak = max(peak, float(f[0]))
                    force_sum += max(0.0, f[0])
                    if f[0] > 0.02:
                        hits.append(c.pos.copy())
                if c.dist < 0:
                    names = [
                        m.body(int(m.geom_bodyid[g])).name or m.geom(int(g)).name
                        for g in (c.geom1, c.geom2)
                    ]
                    key = " / ".join(sorted(names))
                    penetration[key] = min(penetration.get(key, 0), float(c.dist))
        if hits:
            ink.append([i, ids[i], *np.mean(hits, axis=0)])
        mujoco.mj_forward(m, d)
        recorded.append(
            (d.qpos.copy(), d.qvel.copy(), d.site_xpos[tip].copy(), force_sum / 10, bool(hits))
        )
        if i % 500 == 0:
            print("STEP", i, len(targets), "error_mm", np.linalg.norm(error) * 1000, flush=True)
        if not np.isfinite(d.qpos).all() or any(w.number for w in d.warning):
            raise RuntimeError("Nonfinite simulation or MuJoCo warning")
    pos = np.asarray([r[2] for r in recorded])
    hit = np.asarray([r[4] for r in recorded])
    xy = np.linalg.norm(pos[:, :2] - targets[:, :2], axis=1) * 1000
    metrics = {
        "text": text,
        "frames": len(targets),
        "duration_s": len(targets) / 100,
        "mean_pen_down_xy_error_mm": float(xy[down].mean()),
        "p95_pen_down_xy_error_mm": float(np.percentile(xy[down], 95)),
        "pen_down_contact_fraction": float(hit[down].mean()),
        "pen_up_contact_frames": int(hit[~down].sum()),
        "peak_nib_normal_force_N": peak,
        "minimum_contact_distance_by_body_m": penetration,
        "reference_max_position_error_m": max(errors),
        "scope": "Fixed base, gravity-compensated links, rigid pen fixture, compliant nib; not balance, autonomous grasp or hardware validation.",
        "ink_source": "Actual nib-paper contacts, including unintended pen-up contacts",
    }
    names = [m.joint(j).name for j in range(m.njnt)]
    np.savez_compressed(
        out / "rollout.npz",
        qpos=[r[0] for r in recorded],
        qvel=[r[1] for r in recorded],
        tip=pos,
        force=[r[3] for r in recorded],
        contact=hit,
        ink=np.asarray(ink).reshape(-1, 5),
        target=targets,
        pen_down=down,
        stroke_id=ids,
        time_s=(np.arange(len(targets)) + 1) / 100,
        fps=100,
        joint_names=names,
        qpos_addresses=m.jnt_qposadr,
        qvel_addresses=m.jnt_dofadr,
        units="m rad s N",
        world="right-handed Z-up",
        scene_sha256=hashlib.sha256((out / "scene.xml").read_bytes()).hexdigest(),
    )
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2))
    checks = {
        "reference_position_error_below_2mm": max(errors) < 0.002,
        "mean_pen_down_xy_error_below_1mm": float(xy[down].mean()) < 1,
        "p95_pen_down_xy_error_below_2mm": float(np.percentile(xy[down], 95)) < 2,
        "pen_down_contact_at_least_95_percent": float(hit[down].mean()) >= 0.95,
    }
    (out / "writing_checks.json").write_text(
        json.dumps(
            {
                "checks": checks,
                "writing_tracking_pass": all(checks.values()),
                "scope": "Tracking/contact criteria only; NOT collision, force safety, balance, grasp or hardware acceptance.",
                "smoke_truncated": smoke,
            },
            indent=2,
        )
    )
    print(json.dumps(metrics), flush=True)
    if not all(checks.values()):
        raise RuntimeError(
            "Writing tracking/contact checks failed; see preserved writing_checks.json"
        )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--text", default="Hello, Lab.")
    p.add_argument("--smoke", action="store_true")
    a = p.parse_args()
    run(a.output.resolve(), a.model_dir.resolve(), a.text, a.smoke)


if __name__ == "__main__":
    main()
