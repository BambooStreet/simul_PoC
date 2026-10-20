"""Station dimensions in metres; the base turns in place but never translates.

Pallet and tote figures come from the site measurements supplied by the user:
1200 x 1000 x 123 mm pallet, 500 x 400 x 252 mm tote, and a nested tote that
adds exactly 58 mm because the stack seats without a step.
"""

import numpy as np

PALLET_CENTER = np.array([1.15, 0.0])
PALLET_TOP = 0.123
TOTE_HEIGHT = 0.252
NESTING_PITCH = 0.058

# Moulded hand hole in each short end wall, in tote coordinates. The hook prong is
# inserted here; nothing else on the tote is gripped. The hole has to sit within
# one nesting pitch of the rim, otherwise a nested tote buries its own hand holes
# below the rim of the tote underneath and the hooks cannot be pulled back out.
HANDLE_BOTTOM = TOTE_HEIGHT / 2 - NESTING_PITCH + 0.010
HANDLE_TOP = HANDLE_BOTTOM + 0.040
HANDLE_HALF_WIDTH = 0.058
WALL_Y = 0.2475
WALL_T = 0.0025
TOTE_HALF_X = 0.1975

# 2x2 slots. The Y pitch is set by the hook arms, which must withdraw between
# neighbouring columns, not by the tote footprint alone.
SLOT_PITCH_X = 0.44
SLOT_PITCH_Y = 0.66
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

# The conveyor is behind the robot and feeds totes head-on toward its back, so
# the robot picks facing backwards and then turns in place to the pallet.
BELT_HEADING = np.pi
BELT_TOP = 0.525
BELT_HALF_WIDTH = 0.30
BELT_START_X = -2.15
BELT_END_X = -0.47
PICK = np.array([-0.70, 0.0, BELT_TOP + TOTE_HEIGHT / 2])
FEED_START = -2.05
FEED_WAIT = -1.30
FEED_SPEED = 0.15

# Hand pose carried through the turn, in base coordinates. CARRY[0] keeps the
# swept tote inside the pallet's near stack edge at every heading.
CARRY = np.array([0.44, 0.0, 0.95])
PALLET_HEADING = 0.0
BASE_TURN_RATE = 0.6

# Eight nested layers per slot is the reach ceiling: a ninth would have to be
# carried over the far column higher than the fixed-position arms can hold it.
# docs/requirements.md records the measured limits this comes from.
LAYER_LIMIT = 8
MAX_BOXES = 4 * LAYER_LIMIT
