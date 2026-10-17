"""Render recorded physical states; ink is gated by measured nib-paper contact."""

import argparse
import json
from pathlib import Path

import imageio.v2 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation, Slerp

from . import contact_writing as c


def font_at(size):
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default(size=size)


p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--preview", action="store_true")
a = p.parse_args()
out = a.output
m, d = c.load(out)
z = np.load(out / "rollout.npz")
q = z["qpos"]
stats = z["stats"]
fps = int(z["fps"])
start = int(z["camera_start"])
d.qpos[:] = q[0]
mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, height=720, width=960, max_geom=max(14000, len(q) * 2 + 2000))
opt = mujoco.MjvOption()
opt.geomgroup[3] = 0
cam = mujoco.MjvCamera()
cam.lookat[:] = [0.32, -0.03, 0.91]
cam.azimuth = 130
cam.elevation = -38
cam.distance = 1.55
r.update_scene(d, camera=cam, scene_option=opt)
cc = r.scene.camera
pos0 = (cc[0].pos + cc[1].pos) / 2
fw = c.hw.unit(np.array(cc[0].forward))
up = c.hw.unit(np.array(cc[0].up))
right = c.hw.unit(np.cross(fw, up))
rot0 = Rotation.from_matrix(np.column_stack([right, up, -fw]))
rot1 = Rotation.from_matrix(np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]))
slerp = Slerp([0, 1], Rotation.from_quat([rot0.as_quat(), rot1.as_quat()]))
cid = m.camera("cinematic").id
pen_contact = (stats[:, 3] > 0.005) & z["pen_down"]
segments = []
dots = []
if "ink_contact_samples" in z and len(z["ink_contact_samples"]):
    samples = z["ink_contact_samples"]
    bins = np.floor(samples[:, 0] * fps).astype(int)
    points = []
    for b in np.unique(bins):
        group = samples[bins == b]
        points.append((b, int(group[0, 1]), np.average(group[:, 2:5], axis=0, weights=group[:, 5])))
    for b, sid, point in points:
        point = point.copy()
        point[2] = c.hw.PAPER_Z + 0.00025
        dots.append((b, point))
    for (b1, s1, p1), (b2, s2, p2) in zip(points, points[1:]):
        delta = np.linalg.norm(p2[:2] - p1[:2])
        if s1 == s2 and (b2 - b1 <= 2 or delta < 0.0005) and delta < 0.004:
            p1 = p1.copy()
            p2 = p2.copy()
            p1[2] = p2[2] = c.hw.PAPER_Z + 0.00025
            segments.append((b2, p1, p2))
else:
    for i in range(1, len(q)):
        if pen_contact[i] and pen_contact[i - 1] and z["stroke_id"][i] == z["stroke_id"][i - 1]:
            p1 = stats[i - 1, :3].copy()
            p2 = stats[i, :3].copy()
            if np.linalg.norm(p2[:2] - p1[:2]) < 0.004:
                p1[2] = p2[2] = c.hw.PAPER_Z + 0.00025
                segments.append((i, p1, p2))
frames = (
    sorted(
        set(
            [
                0,
                min(200, len(q) - 1),
                min(600, len(q) - 1),
                min(1800, len(q) - 1),
                min(start, len(q) - 1),
                len(q) - 1,
            ]
        )
    )
    if a.preview
    else range(0, len(q), 4)
)
font = font_at(22)
writer = (
    None
    if a.preview
    else iio.get_writer(
        out / "videos/text_contact_writing.mp4",
        fps=fps / 4,
        codec="libx264",
        quality=8,
        macro_block_size=1,
        ffmpeg_params=["-movflags", "+faststart"],
    )
)
for fi in frames:
    d.qpos[:] = q[fi]
    mujoco.mj_forward(m, d)
    t = c.hw.quintic(np.clip((fi - start) / (3 * fps), 0, 1))
    t = 1.0 if a.preview and fi == len(q) - 1 else t
    m.cam_pos[cid] = pos0 * (1 - t) + np.array([0.390, -0.14, c.hw.PAPER_Z + 0.44]) * t
    quat = slerp(float(t)).as_quat()
    m.cam_quat[cid] = quat[[3, 0, 1, 2]]
    mujoco.mj_forward(m, d)
    r.update_scene(d, camera="cinematic", scene_option=opt)
    for stop, p1, p2 in segments:
        if stop > fi:
            break
        c.hw.add_line(r.scene, p1, p2, radius=0.0005)
    for stop, point in dots:
        if stop > fi:
            break
        g = r.scene.geoms[r.scene.ngeom]
        mujoco.mjv_initGeom(
            g,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.0005] * 3),
            point,
            np.eye(3).ravel(),
            np.array([0.025, 0.035, 0.05, 1.0]),
        )
        g.category = mujoco.mjtCatBit.mjCAT_DECOR
        r.scene.ngeom += 1
    r.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
    im = Image.fromarray(r.render())
    draw = ImageDraw.Draw(im)
    draw.rounded_rectangle((15, 15, 580, 77), radius=9, fill=(18, 26, 33))
    draw.text((28, 22), "DexMate | ASCII contact writing", font=font, fill="white")
    draw.text(
        (28, 51),
        "Contact writing | Assisted pen clamp"
        if "clamp" in json.loads((out / "controller.json").read_text()).get("holding_mode", "")
        else "Physical contact writing | Free pen",
        font=font_at(15),
        fill=(190, 208, 217),
    )
    if a.preview:
        im.save(out / "videos" / f"preview_{fi:05d}.jpg")
    else:
        writer.append_data(np.array(im))
    if fi >= len(q) - 4:
        im.save(out / "videos/final_overhead.jpg")
    if fi % 400 == 0:
        print("RENDER", fi, len(q), flush=True)
if writer:
    writer.close()
r.close()
(out / "ink_render_audit.json").write_text(
    json.dumps(
        {
            "segments": len(segments),
            "contact_dots": len(dots),
            "requested_pen_down_frames": int(z["pen_down"].sum()),
            "measured_contact_pen_down_frames": int(pen_contact.sum()),
            "ink_source": "All recorded positive-force physical nib-paper contact positions, including pen-up contacts; force-weighted per-interval positions projected onto paper for display. No planned strokes are painted.",
        },
        indent=2,
    )
)
