import xml.etree.ElementTree as E
from pathlib import Path

import mujoco

from .config import load as load_config
from .drive import add_actuators, urdf_efforts
from .gripper import add_gripper
from .layout import (
    BELT_END_X,
    BELT_HALF_WIDTH,
    BELT_START_X,
    BELT_TOP,
    HANDLE_BOTTOM,
    HANDLE_HALF_WIDTH,
    HANDLE_TOP,
    MAX_BOXES,
    NESTING_PITCH,
    PALLET_CENTER,
    PALLET_TOP,
    TOTE_HALF_X,
    TOTE_HEIGHT,
    WALL_T,
    WALL_Y,
)
from .props import add_assets, add_pallet_visual, add_tote_visual

ROOT = Path(__file__).resolve().parents[1]

URDF_RELATIVE = ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf"
# Drive wheel radius, so the chassis sits on the floor instead of floating.
BASE_Z = 0.0548
HALF_H = TOTE_HEIGHT / 2
# Slack between a tote's foot and the walls of the tote it nests into.
NEST_CLEARANCE = 0.013


def add_tote_walls(body, index):
    """Short end walls carrying the hand hole, plus plain long side walls."""
    for sign in [-1, 1]:
        E.SubElement(
            body,
            "geom",
            name=f"tote_side_{index}_{sign}",
            type="box",
            pos=f"{sign * TOTE_HALF_X} 0 0",
            size=f"{WALL_T} {WALL_Y - WALL_T} {HALF_H}",
            mass=".12",
            rgba="0 0 0 0",
            group="3",
            contype="4",
            conaffinity="2",
        )
        # The lower pane is the ledge the prong rests on once it is seated.
        for part, z0, z1, x0, x1 in [
            ("handle", -HALF_H, HANDLE_BOTTOM, -TOTE_HALF_X, TOTE_HALF_X),
            ("above", HANDLE_TOP, HALF_H, -TOTE_HALF_X, TOTE_HALF_X),
            ("left", HANDLE_BOTTOM, HANDLE_TOP, -TOTE_HALF_X, -HANDLE_HALF_WIDTH),
            ("right", HANDLE_BOTTOM, HANDLE_TOP, HANDLE_HALF_WIDTH, TOTE_HALF_X),
        ]:
            E.SubElement(
                body,
                "geom",
                name=f"tote_{part}_{index}_{sign}",
                type="box",
                pos=f"{(x0 + x1) / 2} {sign * WALL_Y} {(z0 + z1) / 2}",
                size=f"{(x1 - x0) / 2} {WALL_T} {(z1 - z0) / 2}",
                mass=".04",
                rgba="0 0 0 0",
                group="3",
                contype="4",
                conaffinity="2",
            )


def build(count=MAX_BOXES, nesting_pitch=NESTING_PITCH, mode="kinematic",
          config=None):
    """Compile the scene. `mode` decides timestep and whether actuators exist.

    The two modes write different files so a dynamic run cannot leave a scene
    behind that the kinematic tooling then reads as its own.
    """
    if config is None:
        config = load_config("dynamics")
    timestep = config[f"timestep.{mode}"]
    r = E.parse(ROOT / "assets/vega.xml").getroot()
    E.SubElement(
        r, "option", timestep=str(timestep), integrator="implicitfast",
        gravity="0 0 -9.81",
    )
    visual = E.SubElement(r, "visual")
    E.SubElement(visual, "global", offwidth="1280", offheight="960")
    add_assets(r.find("asset"))
    w = r.find("worldbody")
    base = w.find("body[@name='base']")
    base.set("pos", f"0 0 {BASE_Z}")
    # Steered wheels turn the robot in place; the chassis never translates.
    E.SubElement(base, "joint", name="base_yaw", type="hinge", axis="0 0 1", pos="0 0 0")
    for g in base.iter("geom"):
        g.set("rgba", "0.7 0.74 0.8 1")
        g.set("contype", "2")
        g.set("conaffinity", "7")
    contact = E.SubElement(r, "contact")
    srdf = E.parse(
        ROOT / ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.srdf"
    ).getroot()
    for pair in srdf.findall("disable_collisions"):
        E.SubElement(
            contact, "exclude", body1=pair.get("link1"), body2=pair.get("link2")
        )
    for side in ["L", "R"]:
        b = base.find(f".//body[@name='{side}_ee']")
        add_gripper(b, side)
    E.SubElement(w, "light", pos="0 -1 5", dir="0 0 -1", diffuse=".8 .8 .8")
    E.SubElement(
        w,
        "geom",
        name="floor",
        type="plane",
        size="5 5 .1",
        rgba=".19 .22 .26 1",
        contype="1",
        conaffinity="7",
    )
    E.SubElement(
        w,
        "geom",
        name="pallet",
        type="box",
        pos=f"{PALLET_CENTER[0]} {PALLET_CENTER[1]} {PALLET_TOP / 2}",
        size=f".5 .6 {PALLET_TOP / 2}",
        rgba="0 0 0 0",
        group="3",
        contype="1",
        conaffinity="7",
    )
    add_pallet_visual(w, pos=(PALLET_CENTER[0], PALLET_CENTER[1], 0))
    belt_x = (BELT_START_X + BELT_END_X) / 2
    E.SubElement(
        w,
        "geom",
        name="conveyor",
        type="box",
        pos=f"{belt_x} 0 {BELT_TOP / 2}",
        size=f"{(BELT_END_X - BELT_START_X) / 2} {BELT_HALF_WIDTH} {BELT_TOP / 2}",
        rgba=".22 .25 .29 1",
        contype="1",
        conaffinity="7",
    )
    roller = BELT_START_X + 0.1
    while roller < BELT_END_X:
        E.SubElement(
            w,
            "geom",
            type="cylinder",
            pos=f"{roller} 0 {BELT_TOP - 0.025}",
            size=f".025 {BELT_HALF_WIDTH - 0.02}",
            quat=".7071068 .7071068 0 0",
            rgba=".5 .55 .6 1",
            contype="0",
            conaffinity="0",
        )
        roller += 0.2
    E.SubElement(
        w, "camera", name="overview", pos="3.8 -4.8 3.5", xyaxes=".8 .6 0 -.3 .4 .866"
    )
    E.SubElement(
        w,
        "camera",
        name="overhead",
        pos=f"{PALLET_CENTER[0]} {PALLET_CENTER[1]} 3.5",
        quat="1 0 0 0",
    )
    eq = E.SubElement(r, "equality")
    for i in range(count):
        b = E.SubElement(w, "body", name=f"tote_{i}", pos=f"{-4 - i} 3 {HALF_H}")
        E.SubElement(b, "freejoint", name=f"tote_joint_{i}")
        # Nesting surrogate. The foot sets the pitch, and it is sized to drop
        # just inside the end walls of the tote below: the user's stack seats
        # without a step, so a column cannot shear sideways the way a flat
        # box-on-box contact would let it.
        E.SubElement(
            b,
            "geom",
            name=f"tote_geom_{i}",
            type="box",
            pos=f"0 0 {-HALF_H + nesting_pitch / 2}",
            size=f"{TOTE_HALF_X - WALL_T - NEST_CLEARANCE} "
                 f"{WALL_Y - WALL_T - NEST_CLEARANCE} {nesting_pitch / 2}",
            mass="1.5",
            rgba="0 0 0 0",
            group="3",
            friction=".8 .01 .001",
            contype="1",
            conaffinity="7",
        )
        add_tote_walls(b, i)
        add_tote_visual(b, i)
        for side in ["L", "R"]:
            E.SubElement(
                eq,
                "weld",
                name=f"grasp_{side}_{i}",
                body1=f"{side}_tool",
                body2=f"tote_{i}",
                active="false",
                solref=".01 1",
            )

    if mode == "dynamic":
        add_actuators(r, urdf_efforts(ROOT / URDF_RELATIVE))
    path = ROOT / ("assets/scene.xml" if mode == "kinematic"
                   else "assets/scene_dynamic.xml")
    E.ElementTree(r).write(path)
    return mujoco.MjModel.from_xml_path(str(path))
