"""Kinematic robot tracking + dynamic tote contact, with guarded IK motion."""

import json
import math
import threading
import time
import xml.etree.ElementTree as E

import mujoco
import numpy as np

from .gripper import INSERT_LIFT, SPAN_HOLD, SPAN_OPEN
from .ik import IK
from .layout import (
    BASE_TURN_RATE,
    BELT_HEADING,
    CARRY,
    FEED_SPEED,
    FEED_START,
    FEED_WAIT,
    LAYER_LIMIT,
    MAX_BOXES,
    NESTING_PITCH,
    PALLET_HEADING,
    PALLET_TOP,
    PICK,
    SLOTS,
    TOTE_HEIGHT,
)
from .config import load as load_config
from .drive import DynamicDrive, KinematicDrive, urdf_efforts
from .perception import RIM_OFFSET, OverheadCamera
from .scene import ROOT, URDF_RELATIVE, build

# Carried-tote bottom clearance over the tallest existing column.
STACK_CLEARANCE = 0.05
# Height above a tote's centre at which the open hooks clear its rim, so the hands
# can close back to the transit separation before travelling.
RIM_CLEARANCE = 0.08
# The payload hangs a few millimetres below the commanded hand height, so stop the
# descent short and let it drop into the nest instead of pressing it into the
# column below, which makes each layer creep outward.
RELEASE_GAP = 0.008
# Lift off the conveyor before swinging the payload in toward the carry pose.
BELT_LIFT = 0.16


def heading_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Simulation:
    def __init__(
        self,
        count=MAX_BOXES,
        pitch=NESTING_PITCH,
        viewer=False,
        speed=2.0,
        seed=None,
        selection="random",
        mode="kinematic",
        config=None,
    ):
        if count < 1 or count > MAX_BOXES:
            raise ValueError(f"boxes must be in [1, {MAX_BOXES}]")
        if not 0.015 <= pitch <= 0.25:
            raise ValueError("nesting pitch must be in [0.015, 0.25] m")
        if not math.isfinite(speed) or not 0.1 <= speed <= 20:
            raise ValueError("viewer speed must be in [0.1, 20]")
        if mode not in ["kinematic", "dynamic"]:
            raise ValueError("mode must be kinematic or dynamic")
        self.mode = mode
        self.config = load_config("dynamics") if config is None else config
        self.playback_speed = speed
        self.m = build(count, pitch, mode, self.config)
        self.frame_steps = max(1, round(0.02 * speed / self.m.opt.timestep))
        self.wall_start = None
        self.sim_start = 0.0
        self.penetration_limit = self.config[f"contact.penetration_limit_{mode}"]
        self.tote_contact_limit = self.config["contact.unheld_tote_contact_steps"]
        self.d = mujoco.MjData(self.m)
        self.ik = IK(self.m)
        self.seed = (
            int(np.random.SeedSequence().generate_state(1)[0]) if seed is None else seed
        )
        self.rng = np.random.default_rng(self.seed)
        self.selection = selection
        if selection not in ["random", "ordered"]:
            raise ValueError("Unknown slot policy")
        self.placements = []
        self.camera = None
        self.pitch = pitch
        self.count = count
        self.viewer = None
        self.viewer_threads = []
        self.tick = 0
        self.events = []
        self.attached = None
        self.feed_index = None
        self.feed_x = FEED_START
        self.feed_requested = False
        self.max_error = 0.0
        self.max_speed_ratio = 0.0
        self.max_drift = 0.0
        self.tote_contact_steps = 0
        self.worst_tote_contact = 0.0
        self.tote_contact_totes = set()
        self.heights = [0] * 4
        self.qrobot = self.ik.rest.copy()
        self.nr = int(self.m.joint("tote_joint_0").qposadr[0])
        self.nvrobot = int(self.m.joint("tote_joint_0").dofadr[0])
        self.base_q = int(self.m.joint("base_yaw").qposadr[0])
        self.d.qpos[:] = self.qrobot
        self.base_yaw = PALLET_HEADING
        self.span = SPAN_OPEN
        self.center = self.hand_target(CARRY)
        self.qrobot, err = self.ik.solve(
            self.ik_seed(), self.center, self.base_yaw, self.span, 500
        )
        if err > 0.003:
            raise RuntimeError("Initial IK failed")
        self.d.qpos[: self.nr] = self.qrobot[: self.nr]
        mujoco.mj_forward(self.m, self.d)
        self.vel_limits = np.ones(self.nr) * 2
        src = E.parse(
            ROOT / ".deps/dexmate_urdf/robots/humanoid/vega_1/vega_1.urdf"
        ).getroot()
        for j in src.findall("joint"):
            limit = j.find("limit")
            if limit is not None and limit.get("velocity"):
                self.vel_limits[int(self.m.joint(j.get("name")).qposadr[0])] = float(
                    limit.get("velocity")
                )
        self.vel_limits[self.base_q] = BASE_TURN_RATE
        self.steering, self.wheels = self.rolling_wheels()
        # Rolling is slaved to base_yaw, which BASE_TURN_RATE already paces; pacing
        # it again would let the small rear caster throttle the whole trajectory.
        for adr, _ in self.wheels:
            self.vel_limits[adr] = np.inf
        # Start already steered, so the first waypoint is not a standing-still swerve.
        for adr, angle in self.steering:
            self.qrobot[adr] = angle
        self.d.qpos[: self.nr] = self.qrobot[: self.nr]
        mujoco.mj_forward(self.m, self.d)
        self.robot_bodies = set(range(1, int(self.m.body("tote_0").id)))
        self.tote_collision_masks = []
        self.tote_bodies = {int(self.m.body(f"tote_{i}").id): i for i in range(count)}
        for i in range(count):
            body = self.m.body(f"tote_{i}").id
            geoms = np.flatnonzero(self.m.geom_bodyid == body)
            self.tote_collision_masks.append(
                (geoms, self.m.geom_contype[geoms].copy(),
                 self.m.geom_conaffinity[geoms].copy())
            )
            # Unfed totes are inventory, not floor contacts to solve every tick.
            self.m.geom_contype[geoms] = 0
            self.m.geom_conaffinity[geoms] = 0
            self.m.body_gravcomp[body] = 1
        mujoco.mj_forward(self.m, self.d)
        if viewer:
            from mujoco import viewer as mj_viewer

            existing = {t.ident for t in threading.enumerate()}
            self.viewer = mj_viewer.launch_passive(self.m, self.d)
            self.viewer_threads = [
                t for t in threading.enumerate() if t.ident not in existing
            ]
            self.viewer.cam.lookat[:] = [0.4, 0.0, 0.8]
            self.viewer.cam.distance = 5.2
            self.viewer.cam.azimuth = 128
            self.viewer.cam.elevation = -22
        self.validate_contacts()
        self.base_initial = self.d.body("base").xpos.copy()
        if mode == "kinematic":
            self.drive = KinematicDrive(self)
        else:
            self.drive = DynamicDrive(
                self, self.config, urdf_efforts(ROOT / URDF_RELATIVE)
            )
            if self.penetration_limit is None:
                self.log("CONTACT_LIMIT_UNSET",
                         note="dynamic penetration is measured, not enforced")
        for entry in self.config.unevidenced():
            self.log("CONFIG_UNEVIDENCED", path=entry["path"],
                     value=entry["value"], source=entry["source"])

    def rolling_wheels(self):
        """Steer each wheel onto the turn tangent, then roll it without side slip.

        Vega-1 steers: every wheel carries a yaw joint (`*_wheel_j1`) under the
        rolling joint (`*_wheel_j2`). Leaving the yaw at zero drags the tyres
        sideways, because the base pivots 185 mm behind the drive axle rather than
        on it. Turning in place keeps each contact point on a fixed circle about
        the base origin, so in the base frame the tangent direction never moves:
        the steer angle is a constant and only the rolling angle tracks the
        heading. The rear caster trails 34 mm behind its own steer axis, so
        steering swings its contact point; solving iteratively lets that settle.
        """
        jac = np.zeros((3, self.m.nv))
        steering, rolling, slip = [], [], 0.0
        for steer, roll in [
            ("L_wheel_j1", "L_wheel_j2"),
            ("R_wheel_j1", "R_wheel_j2"),
            ("B_wheel_j1", "B_wheel_j2"),
        ]:
            pivot = self.m.joint(steer)
            adr = int(pivot.qposadr[0])
            joint = self.m.joint(roll)
            body = int(self.m.jnt_bodyid[joint.id])
            geom = int(np.flatnonzero(self.m.geom_bodyid == body)[0])
            dof = int(joint.dofadr[0])
            angle = 0.0
            for _ in range(60):
                self.d.qpos[adr] = angle
                mujoco.mj_forward(self.m, self.d)
                # The chassis stands on its wheels, so the axle height is the radius.
                ground = self.d.geom_xpos[geom] * [1, 1, 0]
                mujoco.mj_jac(self.m, self.d, jac, None, ground, body)
                spin = jac[:, dof]
                drift = np.cross([0, 0, 1.0], ground - self.d.body("base").xpos)
                # Rolling runs either way, so align the wheel plane modulo pi and
                # let the rolling ratio pick up the sign.
                error = math.atan2(drift[1], drift[0]) - math.atan2(spin[1], spin[0])
                error = (error + math.pi / 2) % math.pi - math.pi / 2
                angle -= error
                if abs(error) < 1e-12:
                    break
            else:
                raise RuntimeError(f"Steering for {steer} did not converge")
            limit = self.m.jnt_range[pivot.id]
            if self.m.jnt_limited[pivot.id] and not limit[0] <= angle <= limit[1]:
                raise RuntimeError(
                    f"Steering {steer}={angle:.4f} rad outside {np.round(limit, 4)}"
                )
            ratio = -float(spin @ drift) / float(spin @ spin)
            # Whatever the rolling cannot cancel across the wheel plane is scrub.
            # Only the horizontal part is slip: the generated meshes leave each axle
            # a few microradians off level, so the residual keeps a vertical term of
            # about a micrometre per radian that no steering can remove.
            heading = spin * [1, 1, 0]
            heading /= np.linalg.norm(heading)
            residual = drift + ratio * spin
            slip = max(slip, abs(float(residual @ np.cross([0, 0, 1.0], heading))))
            steering.append((adr, float(angle)))
            rolling.append((int(joint.qposadr[0]), ratio))
        if slip > 1e-9:
            raise RuntimeError(f"Wheel side slip {slip:.3e} m per rad of yaw")
        self.wheel_slip = slip * BASE_TURN_RATE
        return steering, rolling

    def hand_target(self, base_point, yaw=None):
        """Base-frame hand centre expressed in world coordinates."""
        return heading_rotation(self.base_yaw if yaw is None else yaw) @ np.asarray(
            base_point, dtype=float
        )

    def ik_seed(self):
        """Commanded robot pose plus the live scene, so IK sees the real obstacles."""
        seed = self.qrobot.copy()
        seed[self.nr :] = self.d.qpos[self.nr :]
        return seed

    def close(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self.viewer is None:
            return
        self.viewer.close()
        # MuJoCo 3.13 close() requests exit but does not join its daemon renderer.
        # Finish renderer cleanup before Python tears down GLFW/OpenGL.
        for thread in self.viewer_threads:
            thread.join(timeout=5)
        if any(thread.is_alive() for thread in self.viewer_threads):
            raise RuntimeError("Viewer renderer did not finish shutdown")
        self.viewer = None

    def log(self, state, **kw):
        item = dict(time=round(self.d.time, 3), state=state, **kw)
        self.events.append(item)
        print(json.dumps(item), flush=True)

    def unexpected_tote_contacts(self):
        """Robot touching a tote it is neither carrying nor reaching into.

        `ik.ignore_bodies` already tracks the totes the hooks are allowed to
        touch: the one being picked, which `engage()` requires prong contact
        with, and the one just released, whose hand holes the hooks are still
        withdrawing from. Anything else is the arm hitting the stack.

        Counts force-producing contacts only, so a hook detecting a ledge a few
        millimetres away does not read as a collision.
        """
        hits = []
        for index in range(self.d.ncon):
            contact = self.d.contact[index]
            # MuJoCo's own inclusion test. The hooks carry a 4 mm margin so
            # `engage()` can seat on a ledge without pressing into it, and the
            # solver really does include those contacts, so a hook within 4 mm
            # of a tote counts as touching it.
            if contact.dist >= contact.includemargin:
                continue
            bodies = [
                int(self.m.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)
            ]
            if not any(b in self.robot_bodies for b in bodies):
                continue
            for body in bodies:
                if body in self.tote_bodies and body not in self.ik.ignore_bodies:
                    hits.append((int(index), self.tote_bodies[body],
                                 float(contact.dist)))
        return hits

    def validate_tote_contacts(self):
        """Dynamic-mode collision verdict: contact, not penetration depth.

        Depth depends on solver parameters; what the step actually needs to know
        is whether the arm touched a tote it had no business touching.
        """
        hits = self.unexpected_tote_contacts()
        if not hits:
            return
        self.tote_contact_steps += 1
        worst = min(hit[2] for hit in hits)
        self.worst_tote_contact = min(self.worst_tote_contact, worst)
        for _, tote, _ in hits:
            self.tote_contact_totes.add(tote)
        if self.tote_contact_limit is None:
            return
        if self.tote_contact_steps > self.tote_contact_limit:
            names = sorted({f"tote_{t}" for _, t, _ in hits})
            raise RuntimeError(
                f"Robot hit {', '.join(names)} it is not holding for "
                f"{self.tote_contact_steps} steps, worst gap {worst:.4f} m"
            )

    def validate_contacts(self):
        if self.mode == "dynamic":
            self.validate_tote_contacts()
        if self.penetration_limit is None:
            return
        for index in np.flatnonzero(self.d.contact.dist < -self.penetration_limit):
            c = self.d.contact[int(index)]
            b1 = int(self.m.geom_bodyid[c.geom1])
            b2 = int(self.m.geom_bodyid[c.geom2])
            if b1 in self.robot_bodies or b2 in self.robot_bodies:
                names = [
                    self.m.geom(g).name or self.m.body(self.m.geom_bodyid[g]).name
                    for g in [c.geom1, c.geom2]
                ]
                raise RuntimeError(
                    f"Robot collision: {names}, penetration={c.dist:.4f} m"
                )

    def validate_payload(self):
        if self.attached is None or self.penetration_limit is None:
            return
        bid = self.m.body(f"tote_{self.attached}").id
        for index in np.flatnonzero(self.d.contact.dist < -self.penetration_limit):
            c = self.d.contact[int(index)]
            bodies = [int(self.m.geom_bodyid[g]) for g in [c.geom1, c.geom2]]
            if (
                bid in bodies
                and not any(b in self.robot_bodies for b in bodies)
            ):
                raise RuntimeError(f"Carried tote penetrates obstacle: {c.dist:.4f} m")

    def step(self, q, v=None):
        if self.viewer and self.wall_start is None:
            self.wall_start = time.perf_counter()
            self.sim_start = self.d.time
        self.drive.command(q, v)
        self.update_feed()
        mujoco.mj_forward(self.m, self.d)
        self.validate_contacts()
        self.validate_payload()
        mujoco.mj_step(self.m, self.d)
        self.drive.settle(q)
        mujoco.mj_forward(self.m, self.d)
        if not np.all(np.isfinite(self.d.qpos)):
            raise RuntimeError("Nonfinite state")
        self.tick += 1
        if self.viewer and self.tick % self.frame_steps == 0:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()
            deadline = (
                self.wall_start + (self.d.time - self.sim_start) / self.playback_speed
            )
            remaining = deadline - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)

    def hold(self, seconds):
        still = np.zeros(self.m.nv)
        for _ in range(math.ceil(seconds / self.m.opt.timestep)):
            self.step(self.qrobot, still)

    def reach(self, base_point, yaw=None):
        """Solve one waypoint at the current hook span; pose is in base coordinates."""
        base_yaw = self.base_yaw if yaw is None else yaw
        center = self.hand_target(base_point, base_yaw)
        target, err = self.ik.solve(self.ik_seed(), center, base_yaw, self.span, 450)
        self.max_error = max(self.max_error, err)
        if err > 0.003:
            raise RuntimeError(
                f"Unreachable target: {np.round(center, 3)}, heading={base_yaw:.3f}, "
                f"span={self.span:.3f}, error={err:.4f}"
            )
        target[self.base_q] = base_yaw
        for adr, angle in self.steering:
            target[adr] = angle
        for adr, gain in self.wheels:
            target[adr] = gain * base_yaw
        return target, center

    def move(self, base_point, span=None):
        """Interpolate the hand centre and hook span at the current heading."""
        start, end = self.center.copy(), self.hand_target(base_point)
        startspan = self.span
        endspan = self.span if span is None else span
        duration = max(
            np.linalg.norm(end - start) / 0.18, abs(endspan - startspan) / 0.09, 0.3
        )
        n = max(2, math.ceil(duration / 0.04))
        base_point = np.asarray(base_point, dtype=float)
        startpoint = heading_rotation(-self.base_yaw) @ start
        for t in np.linspace(0, 1, n + 1)[1:]:
            self.span = startspan + (endspan - startspan) * t
            target, self.center = self.reach(startpoint + (base_point - startpoint) * t)
            self.track(target, 0.04)
        self.span = endspan

    def turn(self, heading, base_point):
        """Rotate the chassis in place while the hands hold a fixed base-frame pose."""
        start = self.base_yaw
        n = max(2, math.ceil(abs(heading - start) / BASE_TURN_RATE / 0.04))
        for t in np.linspace(0, 1, n + 1)[1:]:
            yaw = start + (heading - start) * t
            target, self.center = self.reach(base_point, yaw)
            self.base_yaw = yaw
            self.track(target, 0.04)
        self.base_yaw = heading
        self.log("TURN", heading=round(heading, 3))

    def track(self, target, minimum_duration):
        delta = np.abs(target[: self.nr] - self.qrobot[: self.nr])
        dt = max(minimum_duration, float(np.max(delta / self.vel_limits)) * 1.15)
        steps = math.ceil(dt / self.m.opt.timestep)
        old = self.qrobot.copy()
        self.max_speed_ratio = max(
            self.max_speed_ratio,
            float(np.max(delta / (steps * self.m.opt.timestep) / self.vel_limits)),
        )
        # Constant over the ramp, and in dof order: every robot joint is a hinge.
        rate = np.zeros(self.m.nv)
        rate[: self.nvrobot] = (
            target[: self.nr] - old[: self.nr]
        ) / (steps * self.m.opt.timestep)
        for a in np.linspace(0, 1, steps + 1)[1:]:
            self.step(old + (target - old) * a, rate)
        self.qrobot = target.copy()

    def start_feed(self, i):
        self.activate_tote(i)
        self.ik.ignore_bodies.add(int(self.m.body(f"tote_{i}").id))
        self.feed_index = i
        self.feed_x = FEED_START
        self.feed_requested = False
        self.log("FEED", box=i)

    def activate_tote(self, i):
        """Enable gravity/contact when inventory enters the working scene."""
        geoms, contype, affinity = self.tote_collision_masks[i]
        self.m.geom_contype[geoms] = contype
        self.m.geom_conaffinity[geoms] = affinity
        self.m.body_gravcomp[self.m.body(f"tote_{i}").id] = 0

    def update_feed(self):
        if self.feed_index is None:
            return
        i = self.feed_index
        adr = int(self.m.joint(f"tote_joint_{i}").qposadr[0])
        v = int(self.m.joint(f"tote_joint_{i}").dofadr[0])
        stop_x = PICK[0] if self.feed_requested else FEED_WAIT
        self.feed_x = min(stop_x, self.feed_x + FEED_SPEED * self.m.opt.timestep)
        self.d.qpos[adr : adr + 7] = [self.feed_x, PICK[1], PICK[2], 1, 0, 0, 0]
        self.d.qvel[v : v + 6] = 0

    def spawn(self, i):
        if self.feed_index is None:
            self.start_feed(i)
        if self.feed_index != i:
            raise RuntimeError("Conveyor ordering error")
        self.feed_requested = True
        while self.feed_x < PICK[0] - 1e-8:
            self.step(self.qrobot)
        self.hold(0.3)

    def engage(self, i):
        """Couple only once both prongs are seated on their hand-hole ledges."""
        bid = self.m.body(f"tote_{i}").id
        seated = set()
        for contact in self.d.contact:
            for prong, other in [
                (contact.geom1, contact.geom2),
                (contact.geom2, contact.geom1),
            ]:
                name = self.m.geom(prong).name
                if name.endswith("_prong") and self.m.geom(other).name.startswith(
                    f"tote_handle_{i}_"
                ):
                    seated.add(name)
        if not {"L_prong", "R_prong"}.issubset(seated):
            raise RuntimeError(f"Hook seating failed: {sorted(seated)}")
        self.feed_index = None
        for side in ["L", "R"]:
            hand = self.m.body(f"{side}_tool").id
            eq = self.m.equality(f"grasp_{side}_{i}").id
            R = self.d.xmat[hand].reshape(3, 3)
            self.m.eq_data[eq, 3:6] = R.T @ (self.d.xpos[bid] - self.d.xpos[hand])
            inv = np.empty(4)
            rel = np.empty(4)
            mujoco.mju_negQuat(inv, self.d.xquat[hand])
            mujoco.mju_mulQuat(rel, inv, self.d.xquat[bid])
            self.m.eq_data[eq, 6:10] = rel
            self.d.eq_active[eq] = 1
        self.attached = i
        self.log("HOOKS_SEATED", box=i, contacts=sorted(seated))
        self.hold(0.2)
        if i + 1 < self.count:
            self.start_feed(i + 1)

    def observe(self):
        if self.camera is None:
            self.camera = OverheadCamera(self.m, self.pitch)
        observation = self.camera.observe(self.d)
        self.log(
            "CAMERA_OBSERVATION",
            **{k: v for k, v in observation.items() if k != "time"},
        )
        return observation

    def choose_slot(self, observation):
        counts = np.array(observation["counts"])
        if np.any(counts < 0):
            raise RuntimeError("Pallet camera view is occluded; cannot select a slot")
        capacity = min(LAYER_LIMIT, math.ceil(self.count / 4))
        candidates = np.flatnonzero(counts < capacity)
        if not len(candidates):
            raise RuntimeError("No free pallet slot")
        # Keep columns balanced; random choice is among equally low visible columns.
        candidates = candidates[counts[candidates] == min(counts[candidates])]
        slot = int(
            self.rng.choice(candidates) if self.selection == "random" else candidates[0]
        )
        self.log(
            "SELECT_SLOT",
            slot=slot,
            policy=self.selection,
            candidates=candidates.tolist(),
        )
        return slot, int(counts[slot])

    def pick(self, i):
        """Lower the open hooks beside the tote, slide them in, and seat them."""
        self.spawn(i)
        above = self.local([PICK[0], PICK[1], PICK[2] + INSERT_LIFT])
        self.move(above + [0, 0, RIM_CLEARANCE])
        self.move(above + [0, 0, RIM_CLEARANCE], SPAN_OPEN)
        self.move(above)
        self.move(above, SPAN_HOLD)
        self.log("HOOKS_INSERTED", box=i)
        self.move(self.local(PICK))
        self.engage(i)

    def local(self, world_point):
        """World point in the current base frame."""
        return heading_rotation(-self.base_yaw) @ np.asarray(world_point, dtype=float)

    def place(self, i, slot, targetz, travel_z):
        sx, sy = SLOTS[slot]
        release = targetz + RELEASE_GAP
        self.log("PLACE_APPROACH", box=i, slot=slot)
        self.move(self.local([sx, sy, travel_z]))
        self.move(self.local([sx, sy, release]))
        for side in ["L", "R"]:
            self.d.eq_active[self.m.equality(f"grasp_{side}_{i}").id] = 0
        self.attached = None
        self.hold(0.5)
        self.move(self.local([sx, sy, release]), SPAN_OPEN)
        self.log("RELEASE", box=i, slot=slot)
        # Lift the open hooks past the rim before closing back to transit width,
        # which is the only separation the far column is reachable at.
        self.move(self.local([sx, sy, targetz + RIM_CLEARANCE]))
        self.move(self.local([sx, sy, targetz + RIM_CLEARANCE]), SPAN_HOLD)
        self.move(self.local([sx, sy, travel_z]))
        self.ik.ignore_bodies.discard(int(self.m.body(f"tote_{i}").id))

    def run(self):
        observation = self.observe()
        for i in range(self.count):
            slot, level = self.choose_slot(observation)
            # A tall nested column settles a few millimetres, so seat the next tote on
            # the rim the camera actually sees instead of the nominal lattice height.
            # Clamped, so a bad depth frame cannot throw the placement far off.
            nominal = PALLET_TOP + TOTE_HEIGHT / 2 + level * self.pitch
            targetz = nominal
            if level:
                measured = observation["top_z"][slot] - RIM_OFFSET + self.pitch
                targetz = float(np.clip(measured, nominal - 0.025, nominal + 0.025))
            # The carried tote has to clear the tallest column it crosses, and its
            # own column, before it can be lowered into the nest.
            tallest = max(max(observation["counts"]), level)
            top = PALLET_TOP + (
                TOTE_HEIGHT + (tallest - 1) * self.pitch if tallest else 0.0
            )
            travel_z = max(CARRY[2], top + TOTE_HEIGHT / 2 + STACK_CLEARANCE)
            carry = [CARRY[0], CARRY[1], travel_z]
            self.move(carry, SPAN_HOLD)
            self.turn(BELT_HEADING, carry)
            self.pick(i)
            self.move(self.local([PICK[0], PICK[1], PICK[2] + BELT_LIFT]))
            self.move(carry)
            self.turn(PALLET_HEADING, carry)
            self.place(i, slot, targetz, travel_z)
            self.move(carry)
            observation = self.observe()
            if observation["counts"][slot] != level + 1:
                raise RuntimeError("Camera placement verification failed")
            self.heights = observation["counts"].copy()
            self.placements.append(
                {"box": i, "slot": slot, "level": level,
                 "expected": [float(SLOTS[slot][0]), float(SLOTS[slot][1]), targetz]}
            )
            self.log("PLACED", box=i, counts=self.heights.copy())
        self.hold(0.5)
        drift = sorted(
            (
                float(np.linalg.norm(self.d.body(f"tote_{p['box']}").xpos - p["expected"])),
                p["box"],
            )
            for p in self.placements
        )
        if drift and drift[-1][0] > 0.035:
            worst = ", ".join(f"tote {i}: {d * 1000:.1f} mm" for d, i in drift[-3:][::-1])
            raise RuntimeError(f"Final stack drift: {worst}")
        self.max_drift = drift[-1][0] if drift else 0.0
        self.log("CAPACITY_REACHED", counts=self.heights)
        return self.report()

    def report(self):
        return {
            "boxes": self.count,
            "counts": self.heights,
            "pitch_m": self.pitch,
            "simulation_seconds": self.d.time,
            "max_ik_component_error": self.max_error,
            "max_joint_speed_ratio": self.max_speed_ratio,
            "max_stack_drift_m": self.max_drift,
            "max_wheel_slip_m_s": self.wheel_slip,
            "unheld_tote_contact_steps": self.tote_contact_steps,
            "unheld_tote_contact_worst_gap_m": self.worst_tote_contact,
            "unheld_tote_contact_totes": sorted(self.tote_contact_totes),
            "drive": self.mode,
            "timestep_s": float(self.m.opt.timestep),
            "penetration_limit_m": self.penetration_limit,
            "unevidenced_config": self.config.unevidenced(),
            **self.drive.metrics(),
            "qpos": self.d.qpos.tolist(),
            "base_displacement_m": float(
                np.linalg.norm(self.d.body("base").xpos - self.base_initial)
            ),
            "final_heading_rad": float(self.d.qpos[self.base_q]),
            "seed": self.seed,
            "selection": self.selection,
            "placements": self.placements,
            "positions": [
                self.d.body(f"tote_{i}").xpos.tolist() for i in range(self.count)
            ],
            "mode": "turning base on steered wheels; dual hand-hole hooks; "
                    f"RGB-D semantic camera; {self.mode} robot",
            "events": self.events,
        }
