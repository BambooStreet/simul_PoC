"""E2 tote gripper: one rigid printed piece per wrist, no actuated joints.

Seen from the side it is a lying C. An outer vertical plate stands clear of the
tote wall, a thin tongue runs inward into the hand hole at the bottom, and an
upper arm reaches over the rim with three teeth on its end:

      +-------------------+   upper arm, teeth at the far end
      |
      |   outer plate, outside the collar
      |
      +-----+                 tongue, into the hand hole

That shape is what the measured tote forces. The collar stands 23 mm proud of
the wall, so a column hugging the wall would foul it and the plate has to stand
outside instead. A nested tote leaves only 11 mm of its hand hole clear of the
rim below, so the tongue is 6 mm thick — thicker and it cannot be withdrawn once
the tote is seated.

The teeth exist because lids arrive closed often enough that the robot has to
open them: they drop into the lid's notch, slide under its free edge and lift it
over. Opening and insertion are one continuous translation, so there is still
nothing to actuate.

Every dimension comes from `config/gripper.json`, so a shape study can sweep
them without touching this file.

Tool frame: +X points inboard toward the tote centre, +Z is up, origin at the
root of the tongue's top face. The mount quaternion turns the official wrist
frame into that convention.
"""

import xml.etree.ElementTree as E

from .config import load
from .layout import COLLAR_OUTER, HANDLE_TOP, TOTE_HALF_Y

GRIPPER = load("gripper")
E2 = {key.split(".", 1)[1]: entry["value"]
      for key, entry in GRIPPER.entries.items() if key.startswith("e2.")}

MOUTH = E2["mouth_height"]
THICK = E2["thickness"]
TONGUE_LENGTH = E2["tongue_length"]
TONGUE_HALF_WIDTH = E2["tongue_half_width"]
ARM_LENGTH = E2["arm_length"]
PLATE_BACK = E2["plate_back"]
PLATE_FRONT = E2["plate_front"]
PLATE_HALF_WIDTH = E2["plate_half_width"]
TOOTH_WIDTH = E2["tooth_width"]
TOOTH_HEIGHT = E2["tooth_height"]
TOOTH_FOOT_LENGTH = E2["tooth_foot_length"]
TOOTH_FOOT_THICKNESS = E2["tooth_foot_thickness"]
TOOTH_INSET = E2["tooth_inset"]
TOOTH_HALF_DEPTH = E2["tooth_half_depth"]
TOOTH_SPACING = E2["tooth_spacing"]
FLANGE_HALF = E2["flange_half"]
FLANGE_THICKNESS = E2["flange_thickness"]
FLANGE_X = E2["flange_x"]
DENSITY = E2["density"]
MOUNT_ROLL = E2["mount_roll"]

TOOTH_X = ARM_LENGTH - TOOTH_INSET
TOOTH_BOTTOM = MOUTH + 0.003 - TOOTH_HEIGHT

# Where the tool origin sits is decided by the collar, not by the wall. The lip
# stands 23 mm proud of the hand-hole wall, so the outer plate has to stand clear
# of the lip; pushing it in any further buries the plate in the collar. Measured
# by sweeping the span against the compiled tote: the plate fouls the wall below
# 210 mm and the collar between 210 and 240 mm, and clears from 245 mm out.
PLATE_CLEARANCE = 0.003
SPAN_HOLD = COLLAR_OUTER + PLATE_CLEARANCE + PLATE_FRONT
# How far the tongue reaches past the wall's inner face once seated. This is what
# stops the tote sliding off the tongue, so it is worth reporting.
ENGAGE_DEPTH = TONGUE_LENGTH - (SPAN_HOLD - TOTE_HALF_Y)
# Opening far enough to take the tongue tip outside the wall, plus a margin.
RETRACT = ENGAGE_DEPTH + 0.006
SPAN_OPEN = SPAN_HOLD + RETRACT
# Grip site height relative to the tote centre. The tongue's top face carries the
# load against the top edge of the hand hole; it goes in a tongue-thickness below
# that edge and the tote settles onto it when lifted.
GRIP_Z = HANDLE_TOP - THICK / 2
# The hole leaves 11 mm clear once nested, so there is no room to lift the tongue
# into the middle of the hole the way the old hook did.
INSERT_LIFT = 0.0

MOUNT = {"L": "0.7071068 0.7071068 0 0", "R": "0.7071068 -0.7071068 0 0"}

# One set of boxes is both the collision shell and what is rendered, so the tool
# needs no separate visual mesh.
SOLID = {"group": "2", "contype": "2", "conaffinity": "5", "rgba": ".18 .45 .75 1",
         "friction": ".8 .01 .001", "solref": ".004 1"}


def slab(parent, name, x0, x1, y0, y1, z0, z1, **kw):
    """One box given by its bounds, the way the gripper pack writes them."""
    attrs = {
        "name": name, "type": "box",
        "pos": f"{(x0 + x1) / 2:.5f} {(y0 + y1) / 2:.5f} {(z0 + z1) / 2:.5f}",
        "size": f"{(x1 - x0) / 2:.5f} {(y1 - y0) / 2:.5f} {(z1 - z0) / 2:.5f}",
        "density": f"{DENSITY}",
    }
    attrs.update(SOLID)
    attrs.update(kw)
    return E.SubElement(parent, "geom", **attrs)


def add_gripper(ee, side):
    tool = E.SubElement(ee, "body", name=f"{side}_tool", pos="0 0 0",
                        quat=MOUNT[side])
    slab(tool, f"{side}_plate", PLATE_BACK, PLATE_FRONT,
         -PLATE_HALF_WIDTH, PLATE_HALF_WIDTH, -THICK, MOUTH + THICK)
    # The prong name is kept: engage() looks for it when checking seating.
    slab(tool, f"{side}_prong", PLATE_BACK, TONGUE_LENGTH,
         -TONGUE_HALF_WIDTH, TONGUE_HALF_WIDTH, -THICK, 0.0,
         margin="0.004", gap="0.004")
    slab(tool, f"{side}_arm", PLATE_BACK, ARM_LENGTH,
         -TONGUE_HALF_WIDTH, TONGUE_HALF_WIDTH, MOUTH, MOUTH + THICK)
    slab(tool, f"{side}_flange", FLANGE_X - FLANGE_HALF, FLANGE_X + FLANGE_HALF,
         -FLANGE_HALF, FLANGE_HALF, MOUTH + THICK, MOUTH + THICK + FLANGE_THICKNESS)
    for k, y in enumerate((-TOOTH_SPACING, 0.0, TOOTH_SPACING)):
        slab(tool, f"{side}_tooth_post{k}",
             TOOTH_X - TOOTH_WIDTH / 2, TOOTH_X + TOOTH_WIDTH / 2,
             y - TOOTH_HALF_DEPTH, y + TOOTH_HALF_DEPTH, TOOTH_BOTTOM, MOUTH)
        slab(tool, f"{side}_tooth_foot{k}",
             TOOTH_X - TOOTH_WIDTH / 2 - TOOTH_FOOT_LENGTH,
             TOOTH_X + TOOTH_WIDTH / 2,
             y - TOOTH_HALF_DEPTH, y + TOOTH_HALF_DEPTH,
             TOOTH_BOTTOM, TOOTH_BOTTOM + TOOTH_FOOT_THICKNESS)
    E.SubElement(tool, "site", name=f"{side}_grip", pos="0 0 0", size="0.004",
                 rgba="1 0 0 1")
    E.SubElement(tool, "site", name=f"{side}_tooth", size="0.004",
                 pos=f"{TOOTH_X - TOOTH_WIDTH / 2 - TOOTH_FOOT_LENGTH / 2:.5f} 0 "
                     f"{TOOTH_BOTTOM + TOOTH_FOOT_THICKNESS / 2:.5f}",
                 rgba="1 .5 0 1")
    return tool
