"""Render an actual contact-gated hook pickup and the saved pallet result."""

import json
import sys
from pathlib import Path

import mujoco
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from poc_sim.layout import BELT_HEADING, CARRY, PICK
from poc_sim.perception import OverheadCamera
from poc_sim.scene import ROOT, build
from poc_sim.simulation import BELT_LIFT, Simulation
from scripts.preview_props import render


def main():
    out = ROOT / "reports"
    s = Simulation(1, seed=2)
    try:
        s.turn(BELT_HEADING, CARRY)
        s.pick(0)
        s.move(s.local([PICK[0], PICK[1], PICK[2] + BELT_LIFT]))
        render(
            s.m, s.d, out / "hook_pick.png",
            [PICK[0], PICK[1], PICK[2] + 0.15], 1.5, 235, -22,
        )
    finally:
        s.close()
    report = json.loads((out / "latest.json").read_text())
    m = build(report["boxes"], report["pitch_m"])
    d = mujoco.MjData(m)
    d.qpos[:] = report["qpos"]
    mujoco.mj_forward(m, d)
    render(m, d, out / "turn_base.png", [0.6, 0.0, 0.75], 4.3, 128, -24)
    camera = OverheadCamera(m, report["pitch_m"])
    try:
        result = camera.observe(d)
        assert result["counts"] == report["counts"], result
        Image.fromarray(camera.rgb).save(out / "overhead_camera.png")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
