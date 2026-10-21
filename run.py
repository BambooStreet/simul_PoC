#!/usr/bin/env python
"""Run a finite pallet batch, optionally with the MuJoCo desktop viewer."""

import argparse
import json
import math
from pathlib import Path

import mujoco

from poc_sim.layout import LAYER_LIMIT, MAX_BOXES, NESTING_PITCH
from poc_sim.scene import ROOT
from poc_sim.simulation import Simulation


def save(path, report):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(report, indent=2))


def main():
    p = argparse.ArgumentParser(
        description="Dexmate Vega-1 MuJoCo palletizing baseline"
    )
    p.add_argument("--viewer", action="store_true")
    p.add_argument(
        "--speed",
        type=float,
        default=2.0,
        help="Viewer playback multiplier, 0.1–20 (default: 2); headless runs unpaced",
    )
    p.add_argument(
        "--boxes", type=int, default=MAX_BOXES,
        help=f"Total totes across four slots (default: {MAX_BOXES} = "
             f"{LAYER_LIMIT} layers per slot)",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Set a seed to reproduce the random slot order",
    )
    p.add_argument("--selection", choices=["random", "ordered"], default="random")
    p.add_argument(
        "--mode", choices=["kinematic", "dynamic"], default="kinematic",
        help="kinematic replays the trajectory into the state (the frozen "
             "baseline); dynamic drives the 17 torso and arm joints with torque "
             "actuators (default: kinematic)",
    )
    p.add_argument(
        "--camera-image", help="Save latest overhead RGB sensor frame as PNG"
    )
    p.add_argument(
        "--nesting-pitch",
        type=float,
        default=NESTING_PITCH,
        help="Additional height per tote, metres",
    )
    p.add_argument("--report", default="reports/latest.json")
    p.add_argument(
        "--image", help="Save final overview PNG (MUJOCO_GL=egl for headless)"
    )
    a = p.parse_args()
    if not math.isfinite(a.speed) or not 0.1 <= a.speed <= 20:
        p.error("--speed must be a finite number from 0.1 to 20")
    if not (ROOT / "assets/vega.xml").exists():
        p.error("Robot asset missing. Run bash scripts/setup.sh first.")
    s = Simulation(
        a.boxes,
        a.nesting_pitch,
        a.viewer,
        speed=a.speed,
        seed=a.seed,
        selection=a.selection,
        mode=a.mode,
    )
    try:
        try:
            report = s.run()
        except RuntimeError:
            # Keep the event log and final poses; they are what explains the abort.
            save(a.report, s.report())
            print("FAIL:", a.report)
            raise
        save(a.report, report)
        print("PASS:", a.report)
        if a.camera_image:
            from PIL import Image

            Path(a.camera_image).parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(s.camera.rgb).save(a.camera_image)
        if a.image:
            from PIL import Image

            Path(a.image).parent.mkdir(parents=True, exist_ok=True)
            with mujoco.Renderer(s.m, height=960, width=1280) as renderer:
                renderer.update_scene(s.d, camera="overview")
                Image.fromarray(renderer.render()).save(a.image)
    finally:
        s.close()


if __name__ == "__main__":
    main()
