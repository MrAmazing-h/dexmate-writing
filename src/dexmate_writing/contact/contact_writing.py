"""DexMate contact writing: planned references and simulated, recorded execution."""

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import legacy_writing as hw


def scene(source, out):
    hw.SOURCE = source
    hw.ROOT = out
    (out / "model").mkdir(parents=True, exist_ok=True)
    (out / "videos").mkdir(exist_ok=True)
    hw.build()
    tree = ET.parse(out / "model/scene.xml")
    r = tree.getroot()
    world = r.find("worldbody")
    r.find("option").attrib.update(
        timestep="0.001",
        gravity="0 0 -9.81",
        integrator="implicitfast",
        cone="elliptic",
        iterations="80",
        impratio="10",
    )
    base = r.find('.//body[@name="base"]')
    base.remove(base.find("freejoint"))
    # A stationary writing station. The mobile base is mechanically fixed.
    for b in r.findall(".//body"):
        b.set("gravcomp", "1")
    for j in world.findall(".//joint"):
        if "name" in j.attrib:
            j.set("damping", "0.12")
            j.set("armature", "0.002")
    for g in world.findall("geom"):
        if g.get("name") in ("table_top", "paper") or g.get("type") == "box":
            g.set("contype", "1")
            g.set("conaffinity", "1")
            g.set("friction", ".8 .01 .001")
    for g in r.findall(".//geom"):
        if g.get("contype", "1") != "0":
            g.set("solref", ".004 1")
            g.set("solimp", ".98 .999 .0005")
    hand = r.find('.//body[@name="R_hand_base"]')
    pen = hand.find('body[@name="held_pen"]')
    hand.remove(pen)
    pen.set("name", "free_pen")
    pen.set("gravcomp", "0")
    ET.SubElement(pen, "freejoint", name="pen_free")
    for g in pen.findall("geom"):
        g.set("contype", "1")
        g.set("conaffinity", "1")
        g.set("friction", "1.5 .02 .002")
        g.set("density", "500")
        g.set("condim", "6")
        g.set("solref", ".004 1")
        g.set("solimp", ".98 .999 .0005")
    world.append(pen)
    ET.SubElement(
        hand, "site", name="pen_reference_tip", pos=hw.vec(hw.PEN_TIP), size=".001", rgba="0 0 0 0"
    )
    for a in r.find("actuator"):
        name = a.get("joint")
        kp = (
            500
            if "_arm_" in name
            else (
                1000
                if "torso_" in name
                else 12
                if any("_" + x + "_" in name for x in ["ff", "mf", "rf", "lf", "th"])
                else 100
            )
        )
        a.set("kp", str(kp))
        a.set("kv", str(2 * np.sqrt(kp) * (0.12 if kp < 20 else 0.5)))
        a.set("forcerange", "-120 120" if kp >= 100 else "-1.2 1.2")
    ET.indent(tree)
    tree.write(out / "model/contact_scene.xml", encoding="unicode")


def load(out):
    xml = out / "model/contact_scene.xml"
    if not xml.is_file():
        raise FileNotFoundError(
            f"Missing calibrated contact station: {xml}; use dexmate-contact-write with --station-dir."
        )
    m = mujoco.MjModel.from_xml_path(str(xml))
    return m, mujoco.MjData(m)


def place_pen(m, d, q):
    d.qpos[:] = q
    mujoco.mj_forward(m, d)
    if not any(m.joint(j).name == "pen_free" for j in range(m.njnt)):
        return
    b = m.body("R_hand_base").id
    adr = m.joint("pen_free").qposadr[0]
    sr = m.site("pen_reference_tip").id
    quat = m.site_quat[sr]
    rel = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
    offset = m.site_pos[sr] - rel @ m.site_pos[m.site("pen_tip").id]
    q[adr : adr + 3] = d.xpos[b] + d.xmat[b].reshape(3, 3) @ offset
    quat = Rotation.from_matrix(d.xmat[b].reshape(3, 3) @ rel).as_quat()
    q[adr + 3 : adr + 7] = quat[[3, 0, 1, 2]]
    d.qpos[:] = q
    mujoco.mj_forward(m, d)


def solve_left(m, d, q, target, R):
    js = [m.joint("L_arm_j" + str(i)).id for i in range(1, 8)]
    qa = m.jnt_qposadr[js]
    da = m.jnt_dofadr[js]
    bid = m.body("L_hand_base").id
    jp = np.zeros((3, m.nv))
    jr = jp.copy()
    for _ in range(100):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        rot = d.xmat[bid].reshape(3, 3)
        err = np.r_[target - d.xpos[bid], 0.12 * Rotation.from_matrix(R @ rot.T).as_rotvec()]
        if np.linalg.norm(err) < 1e-6:
            break
        mujoco.mj_jacBody(m, d, jp, jr, bid)
        J = np.vstack((jp[:, da], 0.12 * jr[:, da]))
        dq = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 1e-5, err)
        dq *= min(1.0, 0.15 / max(abs(dq).max(), 1e-6))
        q[qa] = np.clip(q[qa] + dq, m.jnt_range[js, 0] + 0.002, m.jnt_range[js, 1] - 0.002)
    return q, float(np.linalg.norm(err))


def init(out):
    m, d = load(out)
    q = hw.seed(m)
    R = hw.hand_rotation()
    # Clear the writing region; left palm is flat on the left side of the desk.
    for k in ["ff", "mf", "rf", "lf"]:
        hw.setjoint(m, q, "L_" + k + "_j1", 0.0)
    hw.setjoint(m, q, "L_th_j0", 0.7)
    hw.setjoint(m, q, "L_th_j1", 0.0)
    hw.mimic(m, q)
    q, le = solve_left(
        m,
        d,
        q,
        np.array([0.38, 0.25, 0.937]),
        Rotation.from_euler("z", 180, degrees=True).as_matrix(),
    )
    q = hw.solve_pose(m, d, q, np.array([0.40, 0.10, 0.955]), R, iters=300)
    place_pen(m, d, q)
    np.save(out / "initial_qpos.npy", q)
    print("INIT left_error", le, "contacts", d.ncon, flush=True)
    contacts = []
    for c in d.contact:
        if c.dist < 0:
            contacts.append((m.geom(c.geom1).name, m.geom(c.geom2).name, float(c.dist)))
    print(json.dumps(contacts), flush=True)
    return m, d, q


def strokes(out=None):
    if out is not None and (out / "layout.json").exists():
        return [
            (v["line"], np.asarray(v["points"]))
            for v in json.loads((out / "layout.json").read_text())["strokes"]
        ]

    def arc(cx, cy, rx, ry, start=0, end=2 * np.pi, n=30):
        t = np.linspace(start, end, n)
        return np.c_[cx + rx * np.cos(t), cy + ry * np.sin(t)]

    glyph = {
        "Y": ([[(0, 1), (0.4, 0.52), (0.8, 1)], [(0.4, 0.52), (0.4, 0)]], 0.8),
        "u": (
            [
                np.r_[
                    np.array([[0, 0.7], [0, 0.2]]),
                    arc(0.3, 0.2, 0.3, 0.2, np.pi, 2 * np.pi, 12),
                    np.array([[0.6, 0.7], [0.6, 0]]),
                ]
            ],
            0.65,
        ),
        "x": ([[(0, 0.7), (0.6, 0)], [(0.6, 0.7), (0, 0)]], 0.65),
        "i": ([[(0.2, 0.7), (0.2, 0)], [(0.2, 0.96), (0.2, 0.99)]], 0.35),
        "o": ([arc(0.3, 0.35, 0.3, 0.35)], 0.65),
        "n": (
            [
                np.r_[
                    np.array([[0, 0], [0, 0.7], [0, 0.45]]),
                    arc(0.3, 0.45, 0.3, 0.25, np.pi, 0, 12),
                    np.array([[0.6, 0]]),
                ]
            ],
            0.65,
        ),
        "g": (
            [
                arc(0.3, 0.35, 0.3, 0.35),
                np.r_[
                    np.array([[0.6, 0.7], [0.6, -0.15]]), arc(0.3, -0.15, 0.3, 0.2, 0, -np.pi, 12)
                ],
            ],
            0.65,
        ),
        "L": ([[(0, 1), (0, 0), (0.65, 0)]], 0.7),
        "a": ([arc(0.3, 0.35, 0.3, 0.35), [(0.6, 0.7), (0.6, 0)]], 0.65),
        "b": ([[(0, 1), (0, 0)], arc(0.3, 0.35, 0.3, 0.35)], 0.65),
    }
    result = []
    offset = 0
    for ci, ch in enumerate("Yuxiong Lab"):
        if ch == " ":
            offset += 0.5
            continue
        lines, width = glyph[ch]
        for line in lines:
            p = np.array(line, float)
            result.append(
                (
                    ci,
                    np.c_[
                        0.375 + p[:, 1] * 0.031,
                        -0.02 - (offset + p[:, 0]) * 0.031,
                        np.full(len(p), hw.PAPER_Z + 0.0013),
                    ],
                )
            )
        offset += width + 0.2
    return result


def plan(out):
    m, d = load(out)
    q = np.load(out / "calibrated_qpos.npy")
    hw.PEN_TIP = m.site_pos[m.site("pen_reference_tip").id].copy()
    quat = m.site_quat[m.site("pen_reference_tip").id]
    axis = -hw.PEN_AXIS if (out / "reversed_pen.txt").exists() else hw.PEN_AXIS
    hw.PEN_AXIS = Rotation.from_quat(quat[[1, 2, 3, 0]]).apply(axis)
    R = hw.hand_rotation()
    FPS = 100
    pos = []
    down = []
    ids = []

    def move(a, b, seconds, flag=False, sid=-1):
        t = hw.quintic(np.linspace(0, 1, max(2, int(seconds * FPS)), endpoint=False))
        pos.extend(a + (b - a) * t[:, None])
        down.extend([flag] * len(t))
        ids.extend([sid] * len(t))

    start = strokes(out)[0][1][0] + [0, 0, 0.02]
    prev = start
    move(start, start, 1.0)
    for sid, (_, p) in enumerate(strokes(out)):
        up = prev + [0, 0, 0.018]
        move(prev, up, 0.25)
        above = p[0] + [0, 0, 0.018]
        move(up, above, max(0.25, np.linalg.norm(above - up) / 0.08))
        move(above, p[0], 0.35)
        length = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        n = max(30, int(length[-1] / 0.035 * FPS))
        t = hw.quintic(np.linspace(0, 1, n))
        samples = np.column_stack([np.interp(t * length[-1], length, p[:, k]) for k in range(3)])
        pos.extend(samples)
        down.extend([True] * n)
        ids.extend([sid] * n)
        prev = p[-1]
    park = prev + [0, -0.05, 0.08]
    move(prev, prev + [0, 0, 0.04], 0.8)
    move(prev + [0, 0, 0.04], park, 1.5)
    camera_start = len(pos)
    move(park, park, 7.0)
    js = [m.joint("R_arm_j" + str(k)).id for k in range(1, 8)]
    qa = m.jnt_qposadr[js]
    lo = m.jnt_range[js, 0] + 0.002
    hi = m.jnt_range[js, 1] - 0.002
    bid = m.body("R_hand_base").id
    cfg = (
        json.loads((out / "controller.json").read_text())
        if (out / "controller.json").exists()
        else {}
    )
    if "pen_axis_world" in cfg:
        axis_goal = np.array(cfg["pen_axis_world"])
        axis_goal /= np.linalg.norm(axis_goal)
        R = Rotation.align_vectors([axis_goal], [R @ hw.PEN_AXIS])[0].as_matrix() @ R
    collision_geoms = [
        g
        for g in range(m.ngeom)
        if m.geom_contype[g]
        and m.body(m.geom_bodyid[g]).name.startswith(
            ("R_arm_", "R_hand_", "R_th_", "R_ff_", "R_mf_", "R_rf_", "R_lf_")
        )
    ]
    table = m.geom("table_top").id
    initial_target = pos[0].copy()
    flex = cfg.get("flexible_axis", False)
    prior = None

    def initial_error(x):
        row = q.copy()
        row[qa] = x
        d.qpos[:] = row
        mujoco.mj_forward(m, d)
        cur = d.xmat[bid].reshape(3, 3)
        ds = np.array([mujoco.mj_geomDistance(m, d, g, table, 0.02, None) for g in collision_geoms])
        return np.r_[
            initial_target - d.xpos[bid] - cur @ hw.PEN_TIP,
            (0.003 if flex else 0.15) * ((R - cur) @ hw.PEN_AXIS),
            0.0001 * Rotation.from_matrix(R @ cur.T).as_rotvec(),
            4 * np.minimum(ds - 0.008, 0),
            0.1 * min((cur @ hw.PEN_AXIS)[2] - 0.65, 0),
            0.002 * (x - prior) if prior is not None else np.zeros(7),
        ]

    rng = np.random.default_rng(1)
    best = None
    for k in range(24):
        sol = least_squares(
            initial_error,
            np.clip(q[qa], lo, hi) if k == 0 else rng.uniform(lo, hi),
            bounds=(lo, hi),
            max_nfev=240,
        )
        error = np.linalg.norm(initial_error(sol.x))
        if best is None or error < best[0]:
            best = (error, sol.x.copy())
        if error < 1e-6:
            break
    q[qa] = best[1]
    print("INITIAL", best[0], flush=True)
    qs = []
    errors = []
    for i, p in enumerate(pos):
        if flex:
            initial_target = p
            prior = q[qa].copy()
            sol = least_squares(
                initial_error, np.clip(q[qa], lo, hi), bounds=(lo, hi), max_nfev=30, ftol=1e-6
            )
            q[qa] = sol.x
            place_pen(m, d, q)
            qs.append(q.copy())
            errors.append(np.linalg.norm(d.site("pen_reference_tip").xpos - p))
            if i % 500 == 0:
                print("PLAN", i, len(pos), errors[-1], flush=True)
            continue
        q = hw.solve_pose(m, d, q, p, R, iters=30)
        place_pen(m, d, q)
        ds = np.array([mujoco.mj_geomDistance(m, d, g, table, 0.02, None) for g in collision_geoms])
        if ds.min() < 0.005:
            initial_target = p
            sol = least_squares(
                initial_error, np.clip(q[qa], lo, hi), bounds=(lo, hi), max_nfev=100
            )
            q[qa] = sol.x
            place_pen(m, d, q)
        qs.append(q.copy())
        errors.append(np.linalg.norm(d.site("pen_reference_tip").xpos - p))
        if i % 500 == 0:
            print("PLAN", i, len(pos), errors[-1], flush=True)
    np.savez_compressed(
        out / "reference.npz",
        qpos=qs,
        target=pos,
        pen_down=down,
        stroke_id=ids,
        fps=FPS,
        camera_start=camera_start,
        text=json.loads((out / "layout.json").read_text())["text"]
        if (out / "layout.json").exists()
        else "Yuxiong Lab",
    )
    print("PLAN error", max(errors), flush=True)


def simulate(out, seconds=None):
    m, d = load(out)
    z = np.load(out / "reference.npz")
    q = z["qpos"][:, : m.nq]
    fps = int(z["fps"])
    d.qpos[:] = q[0]
    mujoco.mj_forward(m, d)
    js = m.actuator_trnid[:, 0]
    qa = m.jnt_qposadr[js]
    records = []
    stats = []
    maxframes = len(q) if seconds is None else min(len(q), int(seconds * fps))
    cfg = (
        json.loads((out / "controller.json").read_text())
        if (out / "controller.json").exists()
        else {}
    )
    preload = float(cfg.get("grip_preload_m", 0.001))
    correction = np.zeros(m.nq)
    fnames = ["R_th_j0", "R_th_j1", "R_ff_j1", "R_mf_j1"]
    fqa = np.array([m.joint(n).qposadr[0] for n in fnames])
    gids = [
        next(g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name == n)
        for n in ["R_th_l2", "R_ff_l2", "R_mf_l2"]
    ]
    barrel = m.geom("pen_barrel").id

    def distances(row):
        d.qpos[:] = row
        hw.mimic(m, d.qpos)
        mujoco.mj_forward(m, d)
        return np.array([mujoco.mj_geomDistance(m, d, g, barrel, 0.1, None) for g in gids])

    base = distances(q[0])
    J = np.empty((3, 4))
    for k, adr in enumerate(fqa):
        row = q[0].copy()
        row[adr] += 0.0001
        J[:, k] = (distances(row) - base) / 0.0001
    correction[fqa] = np.clip(np.linalg.lstsq(J, -np.ones(3) * preload, rcond=None)[0], -0.15, 0.15)
    # Enforce both actuator-joint and dependent mimic-joint ranges.
    for adr in fqa:
        jid = int(np.flatnonzero(m.jnt_qposadr == adr)[0])
        lower, upper = m.jnt_range[jid].copy()
        for eq in range(m.neq):
            if m.eq_type[eq] == int(mujoco.mjtEq.mjEQ_JOINT) and m.eq_obj2id[eq] == jid:
                child = m.eq_obj1id[eq]
                offset, slope = m.eq_data[eq, :2]
                if slope > 0:
                    lower = max(lower, (m.jnt_range[child, 0] - offset) / slope)
                    upper = min(upper, (m.jnt_range[child, 1] - offset) / slope)
        correction[adr] = (
            np.clip(q[0, adr] + correction[adr], lower + 0.001, upper - 0.001) - q[0, adr]
        )
    # Mild normal preload on the table through the left arm's position servo.
    d.qpos[:] = q[0]
    mujoco.mj_forward(m, d)
    bid = m.body("L_hand_base").id
    jp = np.zeros((3, m.nv))
    jr = jp.copy()
    mujoco.mj_jacBody(m, d, jp, jr, bid)
    ljs = [m.joint("L_arm_j" + str(k)).id for k in range(1, 8)]
    lda = m.jnt_dofadr[ljs]
    L = np.vstack((jp[:, lda], jr[:, lda]))
    correction[m.jnt_qposadr[ljs]] = np.linalg.lstsq(
        L, np.array([0, 0, -0.001, 0, 0, 0]), rcond=None
    )[0]
    d.qpos[:] = q[0]
    d.qpos[fqa] += float(cfg.get("initial_grip_fraction", 0)) * correction[fqa]
    hw.mimic(m, d.qpos)
    mujoco.mj_forward(m, d)
    print("PRELOAD", preload, "finger_delta", correction[fqa], flush=True)
    rjs = [m.joint("R_arm_j" + str(k)).id for k in range(1, 8)]
    rqa = m.jnt_qposadr[rjs]
    rda = m.jnt_dofadr[rjs]
    integral = np.zeros(7)
    rhand = m.body("R_hand_base").id
    previous_force = 0.0
    force_z = 0.0
    max_penetration = 0.0
    deep_contacts = {}
    left_filtered = 0.0
    left_depth = 0.001
    left_direction = correction[m.jnt_qposadr[ljs]].copy() / 0.001
    velocity = np.gradient(q, 1 / fps, axis=0)
    ff = -m.actuator_biasprm[:, 2] / m.actuator_gainprm[:, 0]
    ink = []
    contact_counts = np.zeros(2, int)
    writing_substeps = 0
    pen_contact_writing_steps = 0
    reference_data = mujoco.MjData(m)
    xy_bias = np.zeros(2)
    preload_state = 0.0
    i = 0
    ref_indices = []
    held = 0
    camera_frame = None
    engaged_stroke = -1
    dropout = 0
    for tick in range(maxframes * (5 if cfg.get("contact_phase_control", False) else 1)):
        if i >= maxframes:
            break
        phase_hold = False
        if cfg.get("contact_phase_control", False) and z["pen_down"][i]:
            sid = int(z["stroke_id"][i])
            dropout = dropout + 1 if previous_force < 0.02 else 0
            ready = (
                previous_force > 0.05
                and np.linalg.norm(d.site("pen_tip").xpos[:2] - z["target"][i, :2]) < 0.001
            )
            if engaged_stroke != sid:
                if ready:
                    engaged_stroke = sid
                else:
                    phase_hold = True
            elif dropout > cfg.get("contact_dropout_frames", 5):
                phase_hold = True
        elif not z["pen_down"][i]:
            dropout = 0
        held = held + 1 if phase_hold else 0
        if held > 300:
            raise RuntimeError(
                f"Cannot establish writing contact at reference frame {i}; stopped rather than drawing a missing stroke"
            )
        if camera_frame is None and i >= int(z["camera_start"]):
            camera_frame = len(records)
        if cfg.get("gravity_ramp_seconds", 0) > 0:
            m.opt.gravity[2] = -9.81 * min(1.0, i / (fps * cfg["gravity_ramp_seconds"]))
        target = z["target"][i].copy()
        if cfg.get("hybrid_force_position", False) and i > 50:
            point = d.site("pen_tip").xpos.copy()
            xy_bias = np.clip(xy_bias + 0.03 * (target[:2] - point[:2]), -0.004, 0.004)
            if z["pen_down"][i]:
                force_z = np.clip(
                    force_z
                    + cfg.get("hybrid_force_gain", 0.000006)
                    * (previous_force - cfg.get("hybrid_force_target_N", 1.0)),
                    -0.002,
                    0.0005,
                )
            else:
                force_z *= 0.85
                xy_bias *= 0.99
            desired_preload = cfg.get("hybrid_preload_m", 0.0002) if z["pen_down"][i] else 0.0
            preload_state += cfg.get("preload_ramp_alpha", 1.0) * (desired_preload - preload_state)
            reference_data.qpos[:] = q[i]
            mujoco.mj_forward(m, reference_data)
            mujoco.mj_jac(
                m,
                reference_data,
                jp,
                jr,
                reference_data.site("pen_reference_tip").xpos.copy(),
                rhand,
            )
            J = np.vstack((jp[:, rda], 0.15 * jr[:, rda]))
            err = np.r_[xy_bias, force_z - preload_state, np.zeros(3)]
            integral = np.clip(J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 1e-5, err), -0.1, 0.1)
        elif cfg.get("tip_feedback", False) and i > 50:
            if z["pen_down"][i]:
                force_z = np.clip(
                    force_z
                    + float(cfg.get("force_gain", 0.00004))
                    * (previous_force - float(cfg.get("pen_force_target_N", 0.10))),
                    -0.003,
                    0.002,
                )
            else:
                force_z *= 0.9
            target[2] += force_z
            if z["pen_down"][i]:
                target[2] -= float(cfg.get("pen_preload_m", 0))
            point = d.site("pen_tip").xpos.copy()
            mujoco.mj_jac(m, d, jp, jr, point, rhand)
            J = np.vstack((jp[:, rda], 0.15 * jr[:, rda]))
            err = np.r_[target - point, np.zeros(3)]
            dq = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 1e-5, err)
            integral = np.clip(integral + 0.05 * dq, -0.2, 0.2)
        if cfg.get("support_force_feedback", False):
            left_depth = np.clip(left_depth + 0.000003 * (1.0 - left_filtered), 0, 0.006)
            correction[m.jnt_qposadr[ljs]] = left_direction * left_depth
        command = q[i].copy() + correction
        command[rqa] += integral
        d.ctrl[:] = command[qa]
        if cfg.get("velocity_feedforward", False) and not phase_hold:
            d.ctrl[:] += ff * velocity[i, qa]
        force_sum = np.zeros(3)
        nsub = round(1 / fps / m.opt.timestep)
        for sub in range(nsub):
            mujoco.mj_step(m, d)
            subforces = np.zeros(3)
            force = np.zeros(6)
            for ck, cc in enumerate(d.contact):
                names = [m.geom(g).name for g in [cc.geom1, cc.geom2]]
                bodies = [m.body(m.geom_bodyid[g]).name for g in [cc.geom1, cc.geom2]]
                mujoco.mj_contactForce(m, d, ck, force)
                if "pen_nib" in names and "paper" in names:
                    subforces[0] += force[0]
                    if force[0] > 0.005:
                        ink.append(
                            [float(d.time), int(z["stroke_id"][i]), *cc.pos.copy(), float(force[0])]
                        )
                elif "table_top" in names and any(b.startswith("L_") for b in bodies):
                    subforces[1] += force[0]
                elif "free_pen" in bodies and any(b.startswith("R_") for b in bodies):
                    subforces[2] += force[0]
                max_penetration = max(max_penetration, -float(cc.dist))
                if cc.dist < -0.001:
                    pair = " / ".join(
                        m.body(m.geom_bodyid[g]).name + ":" + m.geom(g).name
                        for g in [cc.geom1, cc.geom2]
                    )
                    deep_contacts[pair] = max(deep_contacts.get(pair, 0.0), -float(cc.dist))
            force_sum += subforces
            contact_counts += subforces[:2] > 0.005
            if z["pen_down"][i]:
                writing_substeps += 1
                pen_contact_writing_steps += int(subforces[0] > 0.005)
        mujoco.mj_forward(m, d)
        tip = d.site("pen_tip").xpos.copy()
        force = np.zeros(6)
        penf = leftf = gripf = 0.0
        bad = 0.0
        contacts = []
        for k, c in enumerate(d.contact):
            b1 = m.body(m.geom_bodyid[c.geom1]).name
            b2 = m.body(m.geom_bodyid[c.geom2]).name
            g1 = m.geom(c.geom1).name
            g2 = m.geom(c.geom2).name
            mujoco.mj_contactForce(m, d, k, force)
            if "pen_nib" in (g1, g2) and "paper" in (g1, g2):
                penf += force[0]
            elif (b1.startswith("L_") or b2.startswith("L_")) and ("table_top" in (g1, g2)):
                leftf += force[0]
            elif "free_pen" in (b1, b2) and (b1.startswith("R_") or b2.startswith("R_")):
                gripf += force[0]
            elif c.dist < bad:
                bad = c.dist
            if c.dist < -0.002:
                contacts.append([g1, g2, float(c.dist)])
        penf, leftf, gripf = force_sum / nsub
        previous_force = penf
        left_filtered = 0.8 * left_filtered + 0.2 * leftf
        records.append(d.qpos.copy())
        stats.append([*tip, penf, leftf, gripf, bad])
        ref_indices.append(i)
        if tick % 100 == 0:
            print(
                "SIM",
                tick,
                "reference",
                i,
                "tip",
                np.round(tip, 3),
                "forces",
                np.round([penf, leftf, gripf], 2),
                "bad",
                round(bad, 5),
                "contacts",
                contacts[:4],
                flush=True,
            )
        if not np.isfinite(d.qpos).all():
            raise RuntimeError("Nonfinite state")
        if not phase_hold:
            i += 1
    if i < maxframes:
        raise RuntimeError("Reference did not finish within the contact-control time budget")
    ref_indices = np.array(ref_indices)
    np.savez_compressed(
        out / "rollout.npz",
        qpos=records,
        stats=stats,
        fps=fps,
        target=z["target"][ref_indices],
        pen_down=z["pen_down"][ref_indices],
        stroke_id=z["stroke_id"][ref_indices],
        camera_start=camera_frame if camera_frame is not None else len(records),
        ink_contact_samples=ink,
        reference_index=ref_indices,
    )
    (out / "physics_contact_coverage.json").write_text(
        json.dumps(
            {
                "writing_contact_physics_steps": pen_contact_writing_steps,
                "writing_physics_steps": writing_substeps,
                "left_contact_physics_steps": int(contact_counts[1]),
                "total_physics_steps": round(d.time / m.opt.timestep),
                "forces_recorded": "average of solver contact forces over each control interval",
            },
            indent=2,
        )
    )
    welds = int(np.sum(m.eq_type == int(mujoco.mjtEq.mjEQ_WELD)))
    pen_free = any(m.joint(j).name == "pen_free" for j in range(m.njnt)) and welds == 0
    (out / "substep_collision_audit.json").write_text(
        json.dumps(
            {
                "dt": m.opt.timestep,
                "max_penetration_m": max_penetration,
                "pairs_exceeding_1mm": deep_contacts,
                "simulation_steps": round(d.time / m.opt.timestep),
                "pen_is_free_body": pen_free,
                "pen_weld_constraints": welds,
                "holding_mode": cfg.get("holding_mode", "finger_friction_only"),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["build", "init", "plan", "simulate"])
    p.add_argument("--source", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seconds", type=float)
    a = p.parse_args()
    if a.mode == "build":
        scene(a.source, a.output)
    elif a.mode == "init":
        init(a.output)
    elif a.mode == "plan":
        plan(a.output)
    elif a.mode == "simulate":
        simulate(a.output, a.seconds)
