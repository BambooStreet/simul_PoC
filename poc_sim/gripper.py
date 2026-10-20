"""Fixed handle hook for the PoC palletiser; there are no actuated jaws.

Each wrist carries one L-shaped printed tool: a bolted plate, a column that runs
down outside the tote's short end wall, and a flat prong that slides into the
moulded hand hole. Picking is pure insertion, so the only commanded "grip" is the
distance between the two prongs. Sizes approximate the photographed part, which
has not been measured.

Tool frame: +X points inboard at the tote centre, +Z is up, and the origin sits on
the wrist plate face. The mount quaternion turns the official wrist frame into
that convention, so the IK reference orientation is a plain world-aligned frame.
"""

import xml.etree.ElementTree as E

from .layout import HANDLE_BOTTOM, WALL_Y
from .props import HOOK_NOSE_X, HOOK_PRONG_X, HOOK_PRONG_Z, hook_tool_geoms

PRONG_HALF_WIDTH = 0.042
PRONG_HALF_HEIGHT = 0.010
# How far past the wall centreline the prong reaches once seated.
ENGAGE_DEPTH = 0.060
# Prong travel that takes the nose from seated to fully outside the tote.
RETRACT = 0.080

# Half-distance between the two prongs, i.e. the commanded hand separation.
SPAN_HOLD = WALL_Y - ENGAGE_DEPTH / 2
SPAN_OPEN = SPAN_HOLD + RETRACT
# Grip site height above the tote centre: the prong resting on the hand-hole ledge.
GRIP_Z = HANDLE_BOTTOM + PRONG_HALF_HEIGHT
# Extra height that centres the prong in the hand hole while it slides in or out.
INSERT_LIFT = 0.010

MOUNT = {"L": "0.7071068 0.7071068 0 0", "R": "0.7071068 -0.7071068 0 0"}


def add_gripper(ee, side):
    tool = E.SubElement(
        ee, "body", name=f"{side}_tool", pos="0 0 0", quat=MOUNT[side]
    )
    for kwargs in hook_tool_geoms(side):
        E.SubElement(tool, "geom", **kwargs)
    # Collision proxies: the printed shell is a visual mesh only.
    for name, pos, size in [
        ("plate", (0.010, 0, -0.013), (0.055, 0.052, 0.013)),
        ("column", (0.0275, 0, -0.105), (0.020, 0.047, 0.079)),
    ]:
        E.SubElement(
            tool,
            "geom",
            name=f"{side}_tool_{name}",
            type="box",
            pos=" ".join(map(str, pos)),
            size=" ".join(map(str, size)),
            mass=".25" if name == "plate" else ".2",
            rgba="0 0 0 0",
            group="3",
            contype="2",
            conaffinity="7",
        )
    half = (HOOK_NOSE_X - 0.0075) / 2
    E.SubElement(
        tool,
        "geom",
        name=f"{side}_prong",
        type="box",
        pos=f"{0.0075 + half} 0 {HOOK_PRONG_Z}",
        size=f"{half} {PRONG_HALF_WIDTH} {PRONG_HALF_HEIGHT}",
        mass=".15",
        rgba="0 0 0 0",
        group="3",
        contype="2",
        conaffinity="7",
        friction="1.5 .01 .001",
        # Seating is verified by proximity, so the hook never has to press into
        # the ledge to register a contact.
        margin=".004",
        gap=".004",
    )
    E.SubElement(
        tool,
        "site",
        name=f"{side}_grip",
        pos=f"{HOOK_PRONG_X} 0 {HOOK_PRONG_Z}",
        size=".006",
        rgba=".1 .9 .4 1",
    )
