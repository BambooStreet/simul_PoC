import unittest

import mujoco
import numpy as np

from poc_sim.gripper import (
    ENGAGE_DEPTH,
    GRIP_Z,
    INSERT_LIFT,
    PLATE_FRONT,
    SPAN_HOLD,
    SPAN_OPEN,
    THICK,
    TONGUE_LENGTH,
)
from poc_sim.ik import REFERENCE_ROT
from poc_sim.layout import (
    BELT_HEADING,
    CARRY,
    COLLAR_OUTER,
    HANDLE_BOTTOM,
    HANDLE_TOP,
    HOLE_CLEAR_HEIGHT,
    LAYER_LIMIT,
    NESTING_PITCH,
    PALLET_TOP,
    PICK,
    SLOTS,
    TOTE_HALF_Y,
    TOTE_HEIGHT,
)
from poc_sim.drive import IDEALISED_JOINTS, TORQUE_JOINTS, urdf_efforts
from poc_sim.scene import ROOT, URDF_RELATIVE, build
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

    def test_tongue_fits_the_part_of_the_hole_a_nested_tote_leaves_clear(self):
        """The measured tote buries most of its own hand hole once nested.

        The hole starts 47 mm below the rim and the nesting pitch is 58 mm, so
        the rim of the tote underneath covers everything below 11 mm. The tongue
        has to live in that band or it cannot be withdrawn after seating.
        """
        rim_below = TOTE_HEIGHT / 2 - NESTING_PITCH
        self.assertLess(HANDLE_BOTTOM, rim_below, "the measured hole is buried")
        self.assertAlmostEqual(HOLE_CLEAR_HEIGHT, HANDLE_TOP - rim_below, places=9)
        tongue_bottom, tongue_top = GRIP_Z - THICK, GRIP_Z
        self.assertGreater(tongue_bottom, rim_below,
                           "tongue would be trapped behind the rim below")
        self.assertLessEqual(tongue_top, HANDLE_TOP)
        self.assertGreater(THICK, 0)
        self.assertLess(THICK, HOLE_CLEAR_HEIGHT)

    def test_opening_pulls_the_tongue_clear_of_the_wall(self):
        self.assertGreater(SPAN_OPEN - TONGUE_LENGTH, TOTE_HALF_Y,
                           "tongue tip still inside the wall when open")
        self.assertGreater(ENGAGE_DEPTH, 0.01, "too little tongue inside the hole")

    def test_plate_stands_outside_the_collar(self):
        """The lip is proud of the hand-hole wall, so the plate cannot hug it."""
        # The plate's inner face is PLATE_FRONT (negative, i.e. outboard) from the
        # tool origin, so it sits further out than the span itself.
        plate_inner = SPAN_HOLD - PLATE_FRONT
        self.assertGreater(plate_inner, COLLAR_OUTER,
                           "plate would be buried in the collar")

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


class DynamicDriveChecks(unittest.TestCase):
    """Step 2: the torque-controlled modes's structure and standing accuracy."""

    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(1, mode="dynamic")

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_every_robot_joint_is_driven(self):
        """An unactuated joint would fall under gravity instead of holding."""
        s = self.sim
        driven = {s.m.actuator(i).trnid[0] for i in range(s.m.nu)}
        robot = {
            j for j in range(s.m.njnt)
            if s.m.jnt_bodyid[j] < s.m.body("tote_0").id
        }
        self.assertEqual(robot, driven)
        self.assertEqual(s.m.nu, len(TORQUE_JOINTS) + len(IDEALISED_JOINTS))

    def test_torque_limits_come_from_the_urdf(self):
        s = self.sim
        efforts = urdf_efforts(ROOT / URDF_RELATIVE)
        for name in TORQUE_JOINTS:
            actuator = s.m.actuator(f"tau_{name}")
            lower, upper = s.m.actuator_forcerange[actuator.id]
            self.assertAlmostEqual(upper, efforts[name], places=6)
            self.assertAlmostEqual(lower, -efforts[name], places=6)

    def test_wrist_gains_account_for_the_payload(self):
        """Sizing on bare link inertia leaves the wrist 20x too soft for a tote."""
        s = self.sim
        # A freshly compiled model has no armature injected, so its mass matrix
        # is the bare link inertia the design sketch sized gains on.
        bare = np.zeros((s.m.nv, s.m.nv))
        bare_model = build(1, s.pitch, "dynamic", s.config)
        bare_data = mujoco.MjData(bare_model)
        bare_data.qpos[:] = s.d.qpos
        mujoco.mj_forward(bare_model, bare_data)
        mujoco.mj_fullM(bare_model, bare_data, bare)
        sized = {j["name"]: j["inertia_kg_m2"] for j in s.drive.metrics()["joints"]}
        for name in ["L_arm_j5", "R_arm_j5"]:
            dof = int(s.m.joint(name).dofadr[0])
            self.assertGreater(sized[name], 10 * bare[dof, dof])

    def test_holding_still_does_not_sag(self):
        s = self.sim
        site = s.m.site("L_grip").id
        before = float(s.d.site_xpos[site][2])
        s.hold(1.0)
        self.assertLess(abs(before - float(s.d.site_xpos[site][2])), 0.002)

    def test_kinematic_mode_has_no_actuators(self):
        """The frozen baseline was recorded with none; adding any would change it."""
        plain = Simulation(1)
        try:
            self.assertEqual(plain.m.nu, 0)
            self.assertEqual(plain.m.opt.timestep, 0.005)
        finally:
            plain.close()


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
