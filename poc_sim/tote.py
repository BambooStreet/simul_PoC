"""The measured Kurly tote: body with hand holes, collar, and two hinged lids.

Built from `config/station.json` rather than from literals, because the 15-layer
review is expected to change the station and because the gripper study sweeps
these dimensions.

Geometry notes that matter downstream:
  * The holed walls face +Y and -Y; the robot reaches them with its two hands.
  * The collar stands 23 mm proud of the body wall and hangs 62 mm below the rim,
    so a neighbouring column occupies more room than the body suggests, and a
    hook hugging the body wall would foul it.
  * The hand hole sits 47 mm below the rim and is 30 mm tall, but a nested tote
    buries all but the top 11 mm behind the rim of the tote below. That is the
    hard limit on how thick a tongue can be.
  * Lid 0 deg is closed and horizontal over the mouth; about 270 deg is folded
    out and down the outside of the wall, which is how they arrive stacked.
"""

import xml.etree.ElementTree as E

from .layout import (
    COLLAR_ARCH_HEIGHT,
    COLLAR_ARCH_HALF_WIDTH,
    COLLAR_DROP,
    COLLAR_OUT,
    HANDLE_BOTTOM,
    HANDLE_TOP,
    HOLE_HALF_WIDTH,
    LID_DEPTH,
    LID_MASS,
    LID_NOTCH_DEPTH,
    LID_NOTCH_WIDTH,
    LID_THICKNESS,
    TOTE_HALF_X,
    TOTE_HALF_Y,
    TOTE_HEIGHT,
    TOTE_MASS,
    WALL_T,
)

# One set of boxes serves as both the collision shell and what the camera sees,
# so they sit in the rendered group rather than being hidden behind a separate
# visual mesh. Contact bits are unchanged from the previous tote.
BLUE = ".11 .17 .45 1"
YELLOW = ".95 .76 .10 1"
SHELL = {"contype": "4", "conaffinity": "2", "rgba": BLUE, "group": "2"}
HALF_H = TOTE_HEIGHT / 2
# Shell panels carry half the body mass; the nesting surrogate carries the rest.
PANEL_MASS = TOTE_MASS / 24


def box(body, name, centre, half, **kw):
    attrs = {
        "name": name, "type": "box",
        "pos": " ".join(f"{v:.5f}" for v in centre),
        "size": " ".join(f"{v:.5f}" for v in half),
        "friction": ".8 .01 .001",
    }
    attrs.update(kw)
    return E.SubElement(body, "geom", **attrs)


def add_body(body, index):
    """Two long walls and the two holed short walls, the latter in four panels.

    No floor plate: the scene's nesting surrogate sits at the bottom of the tote
    and already provides that surface. A full-width floor here would be wider
    than the inner opening of the tote below and would stop the stack nesting.
    """
    for sx in (-1, 1):
        box(body, f"tote_long_{index}_{sx}",
            (sx * (TOTE_HALF_X - WALL_T / 2), 0, 0),
            (WALL_T / 2, TOTE_HALF_Y, HALF_H), mass=f"{PANEL_MASS:.5f}", **SHELL)
    for sy in (-1, 1):
        y = sy * (TOTE_HALF_Y - WALL_T / 2)
        # Four panels around the hand hole: above it, below it, and one each side.
        box(body, f"tote_above_{index}_{sy}", (0, y, (HANDLE_TOP + HALF_H) / 2),
            (TOTE_HALF_X, WALL_T / 2, (HALF_H - HANDLE_TOP) / 2),
            mass=f"{PANEL_MASS:.5f}", **SHELL)
        box(body, f"tote_handle_{index}_{sy}", (0, y, (HANDLE_BOTTOM - HALF_H) / 2),
            (TOTE_HALF_X, WALL_T / 2, (HANDLE_BOTTOM + HALF_H) / 2),
            mass=f"{PANEL_MASS:.5f}", **SHELL)
        side = (TOTE_HALF_X - HOLE_HALF_WIDTH) / 2
        for sx in (-1, 1):
            box(body, f"tote_hole_side_{index}_{sy}_{sx}",
                (sx * (HOLE_HALF_WIDTH + side), y, (HANDLE_TOP + HANDLE_BOTTOM) / 2),
                (side, WALL_T / 2, (HANDLE_TOP - HANDLE_BOTTOM) / 2),
                mass=f"{PANEL_MASS / 2:.5f}", **SHELL)


def add_collar(body, index):
    """The moulded lip. Proud of the wall, and cut away over the hand hole."""
    for sy in (-1, 1):
        y = sy * (TOTE_HALF_Y + COLLAR_OUT / 2)
        side = (TOTE_HALF_X - COLLAR_ARCH_HALF_WIDTH) / 2
        for sx in (-1, 1):
            box(body, f"tote_collar_{index}_{sy}_{sx}",
                (sx * (COLLAR_ARCH_HALF_WIDTH + side), y, HALF_H - COLLAR_DROP / 2),
                (side, COLLAR_OUT / 2, COLLAR_DROP / 2),
                mass=f"{PANEL_MASS / 2:.5f}", **SHELL)
        # Over the hand hole only the top of the lip survives, as an arch.
        box(body, f"tote_collar_{index}_{sy}_arch",
            (0, y, HALF_H - COLLAR_ARCH_HEIGHT / 2),
            (COLLAR_ARCH_HALF_WIDTH, COLLAR_OUT / 2, COLLAR_ARCH_HEIGHT / 2),
            mass=f"{PANEL_MASS / 2:.5f}", **SHELL)


def add_lids(body, index, angle=None):
    """Two hinged lids on the holed walls. 0 is closed, ~270 is folded outside."""
    for sy in (-1, 1):
        lid = E.SubElement(
            body, "body", name=f"tote_lid_{index}_{sy}",
            pos=f"0 {sy * (TOTE_HALF_Y + COLLAR_OUT):.5f} {HALF_H:.5f}",
        )
        E.SubElement(
            lid, "joint", name=f"lid_{index}_{sy}", type="hinge",
            axis=f"{sy} 0 0", range="-20 280", damping="0.02", frictionloss="0.005",
            ref="0" if angle is None else f"{angle:.4f}",
        )
        inward = -sy
        band = LID_DEPTH - LID_NOTCH_DEPTH
        # The hinge-side band runs the full width; the notch only opens the free
        # edge, so the two wings beside it keep the full depth.
        box(lid, f"lid_band_{index}_{sy}",
            (0, inward * band / 2, LID_THICKNESS / 2),
            (TOTE_HALF_X - 0.01, band / 2, LID_THICKNESS / 2),
            mass=f"{LID_MASS / 3:.5f}", rgba=YELLOW, group="2",
            contype="4", conaffinity="2")
        wing = (TOTE_HALF_X - 0.01 - LID_NOTCH_WIDTH / 2) / 2
        for sx in (-1, 1):
            box(lid, f"lid_wing_{index}_{sy}_{sx}",
                (sx * (LID_NOTCH_WIDTH / 2 + wing), inward * LID_DEPTH / 2,
                 LID_THICKNESS / 2),
                (wing, LID_DEPTH / 2, LID_THICKNESS / 2),
                mass=f"{LID_MASS / 3:.5f}", rgba=YELLOW, group="2",
                contype="4", conaffinity="2")


def add_tote(body, index, lid_angle=None):
    add_body(body, index)
    add_collar(body, index)
    add_lids(body, index, lid_angle)
