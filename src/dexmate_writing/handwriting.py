"""Kinematic handwriting animation; fixed pen attachment, not contact-force validation."""

import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path("outputs/writing")
STROKES = Path(__file__).parent / "data/strokes"
SOURCE = Path("external/vega_1p_f5d6")
PAPER_Z = 0.902
TEXT = "小皇帝咩呀天天开心"
PEN_AXIS = np.array([0.75, -0.55, 0.36])
PEN_AXIS /= np.linalg.norm(PEN_AXIS)
PEN_GRASP = np.array([-0.115, -0.027, -0.063])
PEN_TIP = PEN_GRASP - 0.040 * PEN_AXIS
PEN_TOP = PEN_GRASP + 0.130 * PEN_AXIS
PAPER_CENTER = np.array([0.37, -0.105, PAPER_Z])
FPS = 60


def vec(x):
    return " ".join(f"{v:.10g}" for v in x)


def unit(x):
    return x / np.linalg.norm(x)


def attach(parent, tag, **kw):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in kw.items()})


def build():
    tree = ET.parse(SOURCE / "robot.xml")
    root = tree.getroot()
    root.find("compiler").set("meshdir", str(SOURCE))
    root.find("option").set("gravity", "0 0 0")
    vis = root.find("visual")
    vis.find("headlight").set("ambient", ".25 .25 .25")
    vis.find("headlight").set("diffuse", ".35 .35 .35")
    attach(vis, "map", znear=".005", zfar="20")
    asset = root.find("asset")
    attach(
        asset,
        "texture",
        name="floor_tex",
        type="2d",
        builtin="checker",
        rgb1=".18 .22 .25",
        rgb2=".20 .24 .27",
        width="512",
        height="512",
    )
    attach(
        asset, "material", name="floor_mat", texture="floor_tex", texrepeat="4 4", reflectance=".08"
    )
    world = root.find("worldbody")
    ground = world.find("geom[@name='ground']")
    ground.set("material", "floor_mat")
    ground.set("rgba", "1 1 1 1")
    world.find("light[@name='sun']").set("diffuse", ".5 .5 .5")
    world.find("light[@name='sun']").set("ambient", ".15 .15 .15")
    for geom in root.findall(".//geom"):
        if geom.get("type") == "mesh":
            if geom.get("contype") == "0":
                geom.set("rgba", ".65 .70 .73 1")
                if "head_l3" in geom.get("mesh", ""):
                    geom.set("rgba", ".12 .16 .20 1")
            else:
                geom.set("group", "3")
    # Hide collision meshes, preserve the real visual robot geometry.
    attach(
        world,
        "geom",
        name="table_top",
        type="box",
        pos=".59 0 .875",
        size=".32 .43 .025",
        rgba=".43 .28 .16 1",
        contype="0",
        conaffinity="0",
    )
    attach(
        world,
        "geom",
        name="desk_edge",
        type="box",
        pos=".59 0 .844",
        size=".323 .433 .006",
        rgba=".20 .15 .12 1",
        contype="0",
        conaffinity="0",
    )
    for x in [0.32, 0.86]:
        for y in [-0.37, 0.37]:
            attach(
                world,
                "geom",
                type="box",
                pos=vec([x, y, 0.42]),
                size=".024 .024 .42",
                rgba=".12 .15 .18 1",
                contype="0",
                conaffinity="0",
            )
    attach(
        world,
        "geom",
        name="paper",
        type="box",
        pos=vec(PAPER_CENTER - [0, 0, 0.001]),
        size=".09 .25 .001",
        rgba=".985 .975 .925 1",
        contype="0",
        conaffinity="0",
    )
    # Four discreet corner clips, away from the writing field.
    for x in [0.292, 0.448]:
        for y in [-0.339, 0.129]:
            attach(
                world,
                "geom",
                type="box",
                pos=vec([x, y, PAPER_Z + 0.0003]),
                size=".006 .008 .0004",
                rgba=".83 .80 .70 1",
                contype="0",
                conaffinity="0",
            )
    attach(
        world,
        "camera",
        name="cinematic",
        pos=vec(PAPER_CENTER + [0, 0, 0.52]),
        xyaxes="0 -1 0 1 0 0",
        fovy="45",
    )
    attach(
        world,
        "light",
        name="soft_key",
        pos="-.3 -1.5 3.5",
        dir=".25 .4 -1",
        diffuse=".35 .33 .31",
        castshadow="true",
    )
    hand = root.find(".//body[@name='R_hand_base']")
    pen = attach(hand, "body", name="held_pen")
    shaft_start = PEN_TIP + 0.013 * PEN_AXIS
    attach(
        pen,
        "geom",
        name="pen_barrel",
        type="cylinder",
        fromto=vec(np.r_[shaft_start, PEN_TOP]),
        size=".005",
        rgba=".035 .11 .20 1",
        contype="0",
        conaffinity="0",
    )
    attach(
        pen,
        "geom",
        name="pen_grip",
        type="cylinder",
        fromto=vec(np.r_[PEN_TIP + 0.022 * PEN_AXIS, PEN_TIP + 0.065 * PEN_AXIS]),
        size=".0056",
        rgba=".045 .045 .05 1",
        contype="0",
        conaffinity="0",
    )
    attach(
        pen,
        "geom",
        name="pen_nib",
        type="capsule",
        fromto=vec(np.r_[PEN_TIP, PEN_TIP + 0.013 * PEN_AXIS]),
        size=".0013",
        rgba=".10 .10 .11 1",
        contype="0",
        conaffinity="0",
    )
    attach(
        pen,
        "geom",
        name="pen_band",
        type="cylinder",
        fromto=vec(np.r_[PEN_TOP - 0.018 * PEN_AXIS, PEN_TOP - 0.014 * PEN_AXIS]),
        size=".0053",
        rgba=".75 .65 .40 1",
        contype="0",
        conaffinity="0",
    )
    attach(pen, "site", name="pen_tip", pos=vec(PEN_TIP), size=".0001", rgba="0 0 0 0")
    ET.indent(tree, space="  ")
    tree.write(ROOT / "model/scene.xml", encoding="unicode")


def model():
    m = mujoco.MjModel.from_xml_path(str(ROOT / "model/scene.xml"))
    d = mujoco.MjData(m)
    return m, d


def setjoint(m, q, name, v):
    q[m.joint(name).qposadr[0]] = v


def mimic(m, q):
    for i in range(m.neq):
        if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT:
            a, b = m.eq_obj1id[i], m.eq_obj2id[i]
            q[m.jnt_qposadr[a]] = m.eq_data[i, 0] + m.eq_data[i, 1] * q[m.jnt_qposadr[b]]
    return q


def seed(m):
    q = m.qpos0.copy()
    for name, value in [
        ("torso_j1", 0.4),
        ("torso_j2", 0.8),
        ("torso_j3", 0.4),
        ("head_j1", 0.30),
        ("head_j3", 0.15),
        ("R_arm_j4", -1.3),
        ("L_arm_j1", 0.4),
        ("L_arm_j2", 0.15),
        ("L_arm_j4", 0.1),
    ]:
        setjoint(m, q, name, value)
    for side in ["L", "R"]:
        for k in ["ff", "mf", "rf", "lf"]:
            setjoint(
                m,
                q,
                f"{side}_{k}_j1",
                ({"ff": -0.56, "mf": -0.52, "rf": -0.90, "lf": -0.90}[k]) if side == "R" else -0.30,
            )
        setjoint(m, q, side + "_th_j0", 0.6 if side == "R" else 0.9)
        setjoint(m, q, side + "_th_j1", -0.30 if side == "R" else -0.1)
    return mimic(m, q)


def hand_rotation():
    return (
        Rotation.from_euler("y", -45, degrees=True).as_matrix()
        @ Rotation.from_euler("z", 180, degrees=True).as_matrix()
    )


def solve_pose(m, d, q, target, R, side="R", iters=40):
    js = [m.joint(f"{side}_arm_j{k}").id for k in range(1, 8)]
    qa = m.jnt_qposadr[js]
    da = m.jnt_dofadr[js]
    lo = m.jnt_range[js, 0] + 0.001
    hi = m.jnt_range[js, 1] - 0.001
    bid = m.body(side + "_hand_base").id
    jp = np.zeros((3, m.nv))
    jr = np.zeros((3, m.nv))
    offset = PEN_TIP if side == "R" else np.zeros(3)
    for _ in range(iters):
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        curR = d.xmat[bid].reshape(3, 3)
        point = d.xpos[bid] + curR @ offset
        axis = curR @ PEN_AXIS
        goal_axis = R @ PEN_AXIS
        e = np.r_[target - point, 0.15 * (goal_axis - axis)]
        if np.linalg.norm(e) < 2e-6:
            break
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        mujoco.mj_jac(m, d, jp, jr, point, bid)
        J = np.vstack([jp[:, da], -0.15 * cross @ jr[:, da]])
        dq = J.T @ np.linalg.solve(J @ J.T + np.eye(6) * 1e-6, e)
        dq *= min(1.0, 0.12 / max(np.max(np.abs(dq)), 1e-9))
        q[qa] = np.clip(q[qa] + dq, lo, hi)
    return q


def quintic(t):
    return t**3 * (10 - 15 * t + 6 * t * t)


def paths():
    chars = TEXT
    out = []
    size = 0.043
    step = 0.045
    for ci, c in enumerate(chars):
        obj = json.loads((STROKES / f"{c}.json").read_text())
        for si, median in enumerate(obj["medians"]):
            p = np.asarray(median, float)
            dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
            keep = np.r_[True, np.diff(dist) > 1e-6]
            p = p[keep]
            dist = dist[keep] / dist[-1]
            xy = PchipInterpolator(dist, p, axis=0)(np.linspace(0, 1, 180))
            # Human-readable from behind the robot: rightward=-worldY; upward=worldX.
            world = np.column_stack(
                [
                    0.37 + (xy[:, 1] - 400) / 1024 * size,
                    0.075 - ci * step - (xy[:, 0] - 512) / 1024 * size,
                    np.full(len(xy), PAPER_Z + 0.0013),
                ]
            )
            out.append((ci, si, world))
    return out


def trajectory():
    m, d = model()
    q = seed(m)
    R = hand_rotation()
    strokes = paths()
    positions = []
    pens = []
    ids = []
    charids = []

    def add(p, down, sid, cid):
        positions.extend(p)
        pens.extend([down] * len(p))
        ids.extend([sid] * len(p))
        charids.extend([cid] * len(p))

    def travel(a, b, seconds, sid=-1, cid=-1):
        n = max(2, round(seconds * FPS))
        t = quintic(np.linspace(0, 1, n, endpoint=False))
        add(a[None] + t[:, None] * (b - a)[None], False, sid, cid)

    first = strokes[0][2][0]
    start = first + np.array([-0.03, -0.015, 0.055])
    add(np.tile(start, (FPS, 1)), False, -1, -1)
    previous = start
    for sid, (ci, si, p) in enumerate(strokes):
        lift = 0.013
        a = previous + np.array([0, 0, lift]) if sid else previous
        if sid:
            travel(previous, a, 0.15)
        b = p[0] + np.array([0, 0, lift])
        travel(a, b, max(0.24, np.linalg.norm(b - a) / 0.16))
        travel(b, p[0], 0.16)
        length = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        n = max(22, round(length[-1] / 0.095 * FPS))
        s = quintic(np.linspace(0, 1, n))
        sample = np.column_stack([np.interp(s * length[-1], length, p[:, k]) for k in range(3)])
        add(sample, True, sid, ci)
        previous = p[-1]
    writing_end = len(positions)
    end = previous + np.array([0, 0, 0.045])
    travel(previous, end, 0.7)
    rest = end + np.array([-0.19, -0.045, 0.035])
    travel(end, rest, 1.5)
    camera_start = len(positions)
    add(np.tile(rest, (FPS * 9, 1)), False, -1, -1)
    positions = np.asarray(positions)
    js = [m.joint(f"R_arm_j{k}").id for k in range(1, 8)]
    qa0 = m.jnt_qposadr[js]
    lo = m.jnt_range[js, 0] + 0.002
    hi = m.jnt_range[js, 1] - 0.002
    bid = m.body("R_hand_base").id

    def initial_error(x):
        row = q.copy()
        row[qa0] = x
        d.qpos[:] = row
        mujoco.mj_forward(m, d)
        cur = d.xmat[bid].reshape(3, 3)
        return np.r_[
            positions[0] - d.xpos[bid] - cur @ PEN_TIP,
            0.15 * ((R - cur) @ PEN_AXIS),
            0.001 * Rotation.from_matrix(R @ cur.T).as_rotvec(),
        ]

    rng = np.random.default_rng(1)
    best = None
    for k in range(12):
        sol = least_squares(
            initial_error, q[qa0] if k == 0 else rng.uniform(lo, hi), bounds=(lo, hi), max_nfev=220
        )
        err = np.linalg.norm(initial_error(sol.x))
        if best is None or err < best[0]:
            best = (err, sol.x.copy())
        if err < 1e-6:
            break
    q[qa0] = best[1]
    print("INITIAL_IK", best[0], flush=True)
    # Left arm relaxed clear of the desk; no extra moving body parts.
    qa = []
    actual = []
    errors = []
    starttime = time.monotonic()
    for i, target in enumerate(positions):
        q = solve_pose(m, d, q, target, R, iters=12)
        d.qpos[:] = q
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[m.site("pen_tip").id].copy()
        qa.append(q.copy())
        actual.append(tip)
        errors.append(np.linalg.norm(tip - target))
        if i % 500 == 0:
            print(
                "IK",
                i,
                len(positions),
                "error_mm",
                errors[-1] * 1000,
                "seconds",
                round(time.monotonic() - starttime, 1),
                flush=True,
            )
    qa = np.asarray(qa)
    actual = np.asarray(actual)
    names = [m.joint(i).name for i in range(m.njnt)]
    np.savez_compressed(
        ROOT / "writing_reference.npz",
        qpos=qa,
        fps=FPS,
        pen_tip_target=positions,
        pen_tip_actual=actual,
        pen_down=np.array(pens),
        stroke_id=np.array(ids),
        character_index=np.array(charids),
        text=TEXT,
        joint_names=np.array(names),
        joint_qpos_address=m.jnt_qposadr.copy(),
        writing_end_frame=writing_end,
        camera_start_frame=camera_start,
    )
    report = {
        "text": TEXT,
        "frames": len(qa),
        "fps": FPS,
        "duration_seconds": len(qa) / FPS,
        "stroke_count": len(strokes),
        "tip_error_mean_mm": float(np.mean(errors) * 1000),
        "tip_error_max_mm": float(np.max(errors) * 1000),
        "mode": "Kinematic visual demonstration. Pen rigidly attached to palm, preset tripod-style fingers. No physical grasp/contact-force validation. Ink follows actual tip on pen-down frames.",
        "paper_height_m": PAPER_Z,
        "pen_tip_in_right_hand_m": PEN_TIP.tolist(),
        "pen_axis_local": PEN_AXIS.tolist(),
        "qpos_columns": m.nq,
        "writing_end_frame": writing_end,
        "camera_start_frame": camera_start,
        "top_view_hold_seconds": 5,
    }
    (ROOT / "evaluation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


def camera(az=40, elev=-52, dist=1.65):
    c = mujoco.MjvCamera()
    c.lookat[:] = [0.18, -0.04, 1.02]
    c.azimuth = az
    c.elevation = elev
    c.distance = dist
    return c


def add_line(scene, a, b, radius=0.00070):
    if np.linalg.norm(b - a) < 1e-7:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        g,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).ravel(),
        np.array([0.025, 0.035, 0.05, 1.0]),
    )
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)
    g.category = mujoco.mjtCatBit.mjCAT_DECOR
    scene.ngeom += 1


def render(preview=False):
    import imageio.v2 as iio
    from PIL import Image, ImageDraw, ImageFont

    m, d = model()
    z = np.load(ROOT / "writing_reference.npz")
    q = z["qpos"]
    pts = z["pen_tip_actual"]
    downs = z["pen_down"]
    ids = z["stroke_id"]
    d.qpos[:] = q[0]
    mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, height=1080, width=1440, max_geom=16000)
    opt = mujoco.MjvOption()
    opt.geomgroup[3] = 0
    initial_camera = camera()
    renderer.update_scene(d, camera=initial_camera, scene_option=opt)
    # Capture exact starting camera, then interpolate position and rotation to
    # an upright, genuinely vertical overhead camera above the same ink geometry.
    cc = renderer.scene.camera
    initial_pos = (cc[0].pos + cc[1].pos) / 2
    forward = unit(np.asarray(cc[0].forward))
    up = unit(np.asarray(cc[0].up))
    right = unit(np.cross(forward, up))
    initial_rot = Rotation.from_matrix(np.column_stack([right, up, -forward]))
    final_rot = Rotation.from_matrix(np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]))
    rotations = Slerp(
        [0, 1], Rotation.from_quat(np.array([initial_rot.as_quat(), final_rot.as_quat()]))
    )
    final_pos = PAPER_CENTER + np.array([0, 0, 0.52])
    camid = m.camera("cinematic").id
    camera_start = int(z["camera_start_frame"])
    move_frames = 4 * FPS
    frames = (
        [180, 600, 1800, camera_start, camera_start + 120, len(q) - 1]
        if preview
        else range(0, len(q), 2)
    )
    font = ImageFont.load_default(size=28)
    writer = (
        None
        if preview
        else iio.get_writer(
            ROOT / "videos/dexmate_writes.mp4",
            fps=FPS / 2,
            codec="libx264",
            quality=8,
            macro_block_size=1,
            ffmpeg_params=["-movflags", "+faststart"],
        )
    )
    segments = []
    for i in range(1, len(q)):
        if downs[i] and downs[i - 1] and ids[i] == ids[i - 1]:
            a = pts[i - 1].copy()
            b = pts[i].copy()
            a[2] = b[2] = PAPER_Z + 0.00065
            segments.append((i, a, b))
    for fi in frames:
        d.qpos[:] = q[fi]
        mujoco.mj_forward(m, d)
        for ci in [0]:
            t = quintic(np.clip((fi - camera_start) / move_frames, 0, 1))
            m.cam_pos[camid] = initial_pos * (1 - t) + final_pos * t
            xyzw = rotations(float(t)).as_quat()
            m.cam_quat[camid] = xyzw[[3, 0, 1, 2]]
            mujoco.mj_forward(m, d)
            renderer.update_scene(d, camera="cinematic", scene_option=opt)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            for stop, a, b in segments:
                if stop > fi:
                    break
                add_line(renderer.scene, a, b)
            im = Image.fromarray(renderer.render().copy())
            draw = ImageDraw.Draw(im)
            draw.rounded_rectangle((28, 24, 670, 110), radius=12, fill=(17, 25, 32))
            draw.text((48, 30), "DexMate", font=font, fill=(243, 239, 220))
            draw.text((205, 30), "Kinematic writing", font=font, fill=(243, 239, 220))
            draw.text(
                (48, 69),
                "Handwriting  /  simulation",
                font=ImageFont.load_default(size=20),
                fill=(163, 182, 193),
            )
            if preview:
                im.save(ROOT / "videos" / f"preview_{fi:04d}_{ci:02d}.jpg")
            if fi >= len(q) - 2:
                im.save(ROOT / "videos/final_overhead.jpg")
            if not preview:
                writer.append_data(np.asarray(im))
        if fi % 300 == 0:
            print("RENDER", fi, len(q), flush=True)
    if writer:
        writer.close()
    renderer.close()
