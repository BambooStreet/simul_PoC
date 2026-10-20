import unittest

import mujoco
import numpy as np

from poc_sim.gripper import (
    GRIP_Z,
    INSERT_LIFT,
    PRONG_HALF_HEIGHT,
    SPAN_HOLD,
    SPAN_OPEN,
)
from poc_sim.ik import REFERENCE_ROT
from poc_sim.layout import (
    BELT_HEADING,
    CARRY,
    HANDLE_BOTTOM,
    HANDLE_TOP,
    LAYER_LIMIT,
    NESTING_PITCH,
    PALLET_TOP,
    PICK,
    SLOTS,
    TOTE_HEIGHT,
)
from poc_sim.props import HOOK_NOSE_X, HOOK_PRONG_X, RIM_HALF_Y
from poc_sim.simulation import STACK_CLEARANCE, Simulation

REST_Z = PALLET_TOP + TOTE_HEIGHT / 2


class BaselineChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(8)
        for i in range(8):
            cls.sim.activate_tote(i)

    def test_ik_respects_limits_and_rejects_unreachable_target(self):
        s = self.sim
        q, e = s.ik.solve(s.ik.rest, [0.7, 0, 0.6], 0, SPAN_HOLD, 500)
        self.assertLess(e, 0.004)
        self.assertTrue(np.all(q[s.ik.q] >= s.ik.limits[:, 0]))
        self.assertTrue(np.all(q[s.ik.q] <= s.ik.limits[:, 1]))
        _, e = s.ik.solve(s.ik.rest, [5, 0, 1], 0, SPAN_HOLD, 100)
        self.assertGreater(e, 0.1)

    def test_hook_reference_orientation_is_reachable(self):
        """IK drives the tool frame itself, so the reference must be attainable."""
        s = self.sim
        q, e = s.ik.solve(s.ik.rest, CARRY, 0, SPAN_HOLD, 500)
        self.assertLess(e, 0.003)
        s.ik.d.qpos[:] = q
        mujoco.mj_forward(s.m, s.ik.d)
        for k, side in enumerate(["L", "R"]):
            actual = s.ik.d.site_xmat[s.ik.sites[k]].reshape(3, 3)
            self.assertLess(np.abs(actual - REFERENCE_ROT[side]).max(), 0.01, side)

    def test_hand_hole_stays_reachable_once_nested(self):
        """A nested tote must not bury its hand hole under the rim below it."""
        self.assertGreater(
            HANDLE_BOTTOM, TOTE_HEIGHT / 2 - NESTING_PITCH,
            "prong would be trapped under the previous tote's rim",
        )
        self.assertGreaterEqual(GRIP_Z - PRONG_HALF_HEIGHT, HANDLE_BOTTOM)
        self.assertLessEqual(GRIP_Z + INSERT_LIFT + PRONG_HALF_HEIGHT, HANDLE_TOP)
        # Opening the hooks has to pull the nose clear of the widest part of the tote.
        self.assertGreater(SPAN_OPEN - (HOOK_NOSE_X - HOOK_PRONG_X), RIM_HALF_Y)

    def test_top_layer_is_reachable_at_both_spans(self):
        s = self.sim
        top = LAYER_LIMIT - 1
        place_z = REST_Z + top * s.pitch
        travel_z = max(
            CARRY[2],
            PALLET_TOP + TOTE_HEIGHT + top * s.pitch + TOTE_HEIGHT / 2
            + STACK_CLEARANCE,
        )
        for sx, sy in SLOTS:
            for target, span in [
                ([sx, sy, place_z], SPAN_HOLD),
                ([sx, sy, travel_z], SPAN_HOLD),
                ([sx, sy, place_z + INSERT_LIFT], SPAN_OPEN),
            ]:
                _, e = s.ik.solve(s.ik.rest, target, 0, span, 500)
                self.assertLess(e, 0.003, f"layer {top} unreachable at {target}")
        seed = s.ik.rest.copy()
        seed[s.base_q] = BELT_HEADING
        for span in [SPAN_HOLD, SPAN_OPEN]:
            _, e = s.ik.solve(seed, PICK, BELT_HEADING, span, 500)
            self.assertLess(e, 0.003, "conveyor pick unreachable facing backwards")

    def test_obstacle_is_rejected(self):
        s = self.sim
        g = s.m.geom("pallet").id
        old = s.m.geom_pos[g].copy()
        try:
            s.m.geom_pos[g] = [0, 0, 0.20]
            mujoco.mj_forward(s.m, s.d)
            with self.assertRaisesRegex(RuntimeError, "Robot collision"):
                s.validate_contacts()
        finally:
            s.m.geom_pos[g] = old
            mujoco.mj_forward(s.m, s.d)

    def test_payload_obstacle_is_rejected(self):
        s = self.sim
        saved = s.d.qpos.copy()
        try:
            adr = int(s.m.joint("tote_joint_0").qposadr[0])
            s.d.qpos[adr : adr + 7] = [-1.2, 0, 0.30, 1, 0, 0, 0]
            s.attached = 0
            mujoco.mj_forward(s.m, s.d)
            with self.assertRaisesRegex(RuntimeError, "Carried tote penetrates"):
                s.validate_payload()
        finally:
            s.attached = None
            s.d.qpos[:] = saved
            mujoco.mj_forward(s.m, s.d)

    def test_totes_settle_at_nesting_pitch_under_gravity(self):
        s = self.sim
        sx, sy = SLOTS[0]
        for i, z in [(0, REST_Z + 0.03), (1, REST_Z + s.pitch + 0.055)]:
            adr = int(s.m.joint(f"tote_joint_{i}").qposadr[0])
            s.d.qpos[adr : adr + 7] = [sx, sy, z, 1, 0, 0, 0]
        s.hold(1.5)
        a = s.d.body("tote_0").xpos
        b = s.d.body("tote_1").xpos
        self.assertAlmostEqual(a[2], REST_Z, delta=0.003)
        self.assertAlmostEqual(b[2] - a[2], s.pitch, delta=0.003)
        self.assertLess(np.linalg.norm(b[:2] - [sx, sy]), 0.003)


class StationChecks(unittest.TestCase):
    def test_base_turns_in_place_and_holds_still_for_arm_moves(self):
        s = Simulation(1)
        try:
            pos = s.d.body("base").xpos.copy()
            rotation = s.d.body("base").xquat.copy()
            joints = s.d.qpos[s.ik.q].copy()
            s.move(CARRY + [0.05, 0, -0.03])
            np.testing.assert_array_equal(s.d.body("base").xpos, pos)
            np.testing.assert_array_equal(s.d.body("base").xquat, rotation)
            self.assertGreater(np.linalg.norm(s.d.qpos[s.ik.q] - joints), 0.01)
            s.turn(BELT_HEADING, CARRY)
            np.testing.assert_allclose(s.d.body("base").xpos, pos, atol=1e-9)
            self.assertAlmostEqual(float(s.d.qpos[s.base_q]), BELT_HEADING, places=6)
            # Facing backwards, the hands sit behind the robot, over the conveyor.
            self.assertLess(s.d.site_xpos[s.ik.sites[0]][0], -0.2)
            # The drive wheels roll through the turn instead of sliding.
            self.assertGreater(
                min(abs(float(s.d.qpos[adr])) for adr, _ in s.wheels), 1.0
            )
        finally:
            s.close()

    def test_coupling_without_both_hooks_seated_is_rejected(self):
        s = Simulation(1)
        try:
            with self.assertRaisesRegex(RuntimeError, "Hook seating failed"):
                s.engage(0)
            self.assertFalse(np.any(s.d.eq_active))
        finally:
            s.close()

    def test_camera_measures_scene_instead_of_internal_counts(self):
        s = Simulation(3)
        try:
            s.heights = [99, 99, 99, 99]
            for i, slot, level in [(0, 1, 0), (1, 2, 0), (2, 2, 1)]:
                adr = int(s.m.joint(f"tote_joint_{i}").qposadr[0])
                s.d.qpos[adr : adr + 7] = [
                    *SLOTS[slot],
                    REST_Z + level * s.pitch,
                    1,
                    0,
                    0,
                    0,
                ]
            mujoco.mj_forward(s.m, s.d)
            observation = s.observe()
            self.assertEqual(observation["counts"], [0, 1, 2, 0])
            slot, level = s.choose_slot(observation)
            self.assertIn(slot, [0, 3])
            self.assertEqual(level, 0)
            with self.assertRaisesRegex(RuntimeError, "occluded"):
                s.choose_slot({"counts": [-1, 0, 0, 0]})
        finally:
            s.close()


if __name__ == "__main__":
    unittest.main()
