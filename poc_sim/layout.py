"""Station dimensions, all of them read from `config/station.json`.

The 15-layer review changes pallet distance, slot pitch and layer count, so every
figure lives in one JSON file with its own source attached rather than as a
literal here. This module only holds the expressions derived from them.

Tote frame: the two short walls carrying the hand holes face +Y and -Y, because
the robot reaches out to them with its left and right hand. The long walls face
+X and -X. +Z is up and the origin is the body centre. The gripper pack models
the tote the other way round; placing it this way instead keeps a yaw rotation
out of every pose in the simulation.
"""

import numpy as np

from .config import load

STATION = load("station")

PALLET_TOP = STATION["pallet.top"]
PALLET_HALF_X = STATION["pallet.half_x"]
PALLET_HALF_Y = STATION["pallet.half_y"]
PALLET_CENTER = np.array([STATION["pallet.center_x"], 0.0])

# Body, i.e. the walls the hand holes are moulded into. The collar stands proud of
# this, so anything about clearance has to use COLLAR_OUTER instead.
TOTE_HEIGHT = STATION["tote.body_height"]
WALL_T = STATION["tote.wall_thickness"]
NESTING_PITCH = STATION["tote.nesting_pitch"]
TOTE_MASS = STATION["tote.mass"]
# 446 mm separates the holed walls, so it lies along Y; 400 mm along X.
TOTE_HALF_Y = STATION["tote.body_length"] / 2
TOTE_HALF_X = STATION["tote.body_width"] / 2

COLLAR_OUT = STATION["collar.overhang"]
COLLAR_DROP = STATION["collar.drop"]
COLLAR_ARCH_HALF_WIDTH = STATION["collar.arch_half_width"]
COLLAR_ARCH_HEIGHT = STATION["collar.arch_height"]
# What a neighbouring column occupies and an arm has to clear, on the holed side.
COLLAR_OUTER = TOTE_HALF_Y + COLLAR_OUT

HOLE_WIDTH = STATION["hand_hole.width"]
HOLE_HEIGHT = STATION["hand_hole.height"]
HOLE_TOP_BELOW_RIM = STATION["hand_hole.top_below_rim"]
HOLE_HALF_WIDTH = HOLE_WIDTH / 2
# Heights relative to the body centre, so they can be read straight off the tote.
HANDLE_TOP = TOTE_HEIGHT / 2 - HOLE_TOP_BELOW_RIM
HANDLE_BOTTOM = HANDLE_TOP - HOLE_HEIGHT
# A nested tote buries the lower part of its own hand hole behind the rim of the
# tote underneath. This is what is left, and it caps the gripper tongue.
HOLE_CLEAR_HEIGHT = NESTING_PITCH - HOLE_TOP_BELOW_RIM

LID_DEPTH = STATION["lid.depth"]
LID_THICKNESS = STATION["lid.thickness"]
LID_NOTCH_WIDTH = STATION["lid.notch_width"]
LID_NOTCH_DEPTH = STATION["lid.notch_depth"]
LID_MASS = STATION["lid.mass"]
LID_OPEN_ANGLE = STATION["lid.open_angle"]

SLOT_PITCH_X = STATION["slots.pitch_x"]
SLOT_PITCH_Y = STATION["slots.pitch_y"]
LAYER_LIMIT = STATION["slots.layer_limit"]
MAX_BOXES = 4 * LAYER_LIMIT
SLOTS = np.array(
    [
        [
            PALLET_CENTER[0] + sx * SLOT_PITCH_X / 2,
            PALLET_CENTER[1] + sy * SLOT_PITCH_Y / 2,
        ]
        for sx in [1, -1]
        for sy in [-1, 1]
    ]
)

# The conveyor is behind the robot and feeds totes head-on toward its back, so the
# robot picks facing backwards and then turns in place to the pallet.
BELT_HEADING = np.pi
BELT_TOP = STATION["conveyor.top"]
BELT_HALF_WIDTH = STATION["conveyor.half_width"]
BELT_START_X = STATION["conveyor.start_x"]
BELT_END_X = STATION["conveyor.end_x"]
FEED_SPEED = STATION["conveyor.feed_speed"]
PICK = np.array([-0.70, 0.0, BELT_TOP + TOTE_HEIGHT / 2])
FEED_START = -2.05
FEED_WAIT = -1.30

# Hand pose carried through the turn, in base coordinates. CARRY[0] keeps the swept
# tote inside the pallet's near stack edge at every heading.
CARRY = np.array([0.44, 0.0, 0.95])
PALLET_HEADING = 0.0
BASE_TURN_RATE = 0.6
