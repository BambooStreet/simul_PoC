"""Render mesh details and the saved final robot state without replaying motion."""

import argparse
import json
import sys
import xml.etree.ElementTree as E
from pathlib import Path

import mujoco
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from poc_sim.gripper import GRIP_Z, SPAN_HOLD
from poc_sim.layout import NESTING_PITCH, PALLET_TOP, TOTE_HEIGHT
from poc_sim.props import (
    HOOK_PRONG_X,
    HOOK_PRONG_Z,
    add_assets,
    add_pallet_visual,
    add_tote_visual,
    hook_tool_geoms,
)
from poc_sim.scene import ROOT, build


def render(model, data, path, lookat, distance, azimuth, elevation):
    camera = mujoco.MjvCamera()
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = azimuth
    camera.elevation = elevation
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=960, width=1280) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def add_detail_hooks(world, tote_y, tote_z):
    """Both handle hooks, seated in one tote, so the tool shape is visible."""
    for side, sign, quat in [("L", 1, "0.7071068 0 0 -0.7071068"),
                             ("R", -1, "0.7071068 0 0 0.7071068")]:
        y = tote_y + sign * (SPAN_HOLD + HOOK_PRONG_X)
        z = tote_z + GRIP_Z - HOOK_PRONG_Z
        tool = E.SubElement(
            world, "body", name=f"detail_hook_{side}", pos=f"0 {y} {z}", quat=quat
        )
        for kwargs in hook_tool_geoms(side):
            E.SubElement(tool, "geom", **kwargs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="reports")
    args = p.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    root = E.Element("mujoco", model="Tote and pallet visual reference")
    E.SubElement(root, "compiler", angle="radian")
    visual = E.SubElement(root, "visual")
    E.SubElement(visual, "global", offwidth="1280", offheight="960")
    E.SubElement(
        visual,
        "headlight",
        ambient=".35 .35 .35",
        diffuse=".7 .7 .7",
        specular=".3 .3 .3",
    )
    asset = E.SubElement(root, "asset")
    add_assets(asset)
    world = E.SubElement(root, "worldbody")
    E.SubElement(world, "light", pos="-1 -2 4", diffuse=".8 .8 .8", castshadow="true")
    E.SubElement(world, "light", pos="2 1 3", diffuse=".45 .45 .45", castshadow="false")
    E.SubElement(world, "geom", type="plane", size="4 4 .1", rgba=".68 .70 .73 1")
    add_pallet_visual(world, (0, 0, 0))
    for i in range(4):
        # One empty tote plus three nested totes, on the supplied-size pallet.
        y = -0.33 if i == 0 else 0.33
        z = PALLET_TOP + TOTE_HEIGHT / 2 + max(i - 1, 0) * NESTING_PITCH
        body = E.SubElement(world, "body", name=f"detail_tote_{i}", pos=f"0 {y} {z}")
        add_tote_visual(body, i)
        if i == 0:
            add_detail_hooks(world, y, z)
    model = mujoco.MjModel.from_xml_string(E.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    render(model, data, out / "props_detail.png", [0, 0, 0.22], 2.0, 135, -22)
    render(model, data, out / "props_top.png", [0, 0, 0.25], 1.8, 135, -62)
    render(model, data, out / "hook_tool.png", [0, -0.33, 0.33], 1.0, 200, -12)
    report = json.loads((ROOT / "reports/latest.json").read_text())
    model = build(report["boxes"], report["pitch_m"])
    data = mujoco.MjData(model)
    data.qpos[:] = report["qpos"]
    mujoco.mj_forward(model, data)
    render(model, data, out / "scene_updated.png", [0.6, 0.0, 0.75], 4.7, 128, -24)
    print("Saved previews:", out)


if __name__ == "__main__":
    main()
