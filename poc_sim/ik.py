import mujoco
import numpy as np

from .gripper import GRIP_Z

# Reference tool orientation at yaw 0; columns are the tool axes in world
# coordinates. +X points inboard at the tote centre and +Z is up, so a hook can
# slide straight into the hand hole.
REFERENCE_ROT = {
    "L": np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
    "R": np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
}

# The 12-row reach task leaves 5 redundant DOF. Posture and obstacle repulsion ride
# in that nullspace, so they steer the elbows and torso without moving the hands.
POSTURE_GAIN = 0.02
AVOID_ACTIVATE = 0.08
# Repulsion aims for SAFE but only has to clear REQUIRE before a solve may finish;
# SAFE is not always reachable where the arm threads between two full columns, and
# gating on it alone would burn every iteration. REQUIRE has to leave room for the
# joint-space interpolation between waypoints, which is not itself distance-checked.
AVOID_SAFE = 0.045
AVOID_REQUIRE = 0.014
AVOID_GAIN = 0.3
# Nullspace motion holds the hands still only to first order. Keep the secondary step
# small enough that its second-order leak stays well under the convergence threshold.
AVOID_STEP = 0.01
# While the margin is still short of REQUIRE the solve cannot finish anyway, so the
# secondary term may move coarsely and escape a pinned posture in a few iterations.
AVOID_RUSH = 0.05
AVOID_REFRESH = 4
# A full 2x2 pallet puts a few hundred obstacle geoms in broadphase range, so the
# budget has to be wide enough that the pair that matters is not crowded out.
AVOID_PAIRS = 32
BROADPHASE = 0.3


class IK:
    def __init__(self, model):
        self.m = model
        self.d = mujoco.MjData(model)
        self.names = [f"torso_j{i}" for i in range(1, 4)] + [
            f"{s}_arm_j{i}" for s in ["L", "R"] for i in range(1, 8)
        ]
        self.ids = np.array([model.joint(n).id for n in self.names])
        self.q = model.jnt_qposadr[self.ids]
        self.v = model.jnt_dofadr[self.ids]
        self.limits = model.jnt_range[self.ids]
        self.sites = [model.site(s + "_grip").id for s in ["L", "R"]]
        for n, x in [("torso_j1", 1), ("torso_j2", 1.8), ("torso_j3", 0.8)]:
            self.d.qpos[model.joint(n).qposadr] = x
        mujoco.mj_forward(model, self.d)
        self.rest = self.d.qpos.copy()
        self.rot = [REFERENCE_ROT[s] for s in ["L", "R"]]
        for side in ["L", "R"]:
            self.rest[model.joint(f"{side}_arm_j4").qposadr] = -1.2
        # Bodies the carried tote and the tote being picked occupy; the caller keeps
        # this current so the gripper is not repelled from its own payload.
        self.ignore_bodies = set()
        self.moving_geoms, self.moving_bodies = self.movable_geoms()
        env = [
            g
            for g in range(model.ngeom)
            if int(model.geom_bodyid[g]) not in self.moving_bodies
            and self.environment(int(model.geom_bodyid[g]))
        ]
        self.env_geoms = np.array(env, dtype=int)
        self.env_bodies = model.geom_bodyid[self.env_geoms]
        self.clearance = np.inf
        self.jp = np.zeros((3, model.nv))
        self.jr = np.zeros((3, model.nv))
        self.fromto = np.zeros(6)

    def movable_geoms(self):
        """Collidable geoms that the controlled joints can actually displace."""
        controlled = {int(i) for i in self.ids}
        moved = set()
        for b in range(self.m.nbody):
            link = b
            while link > 0:
                adr = int(self.m.body_jntadr[link])
                stop = adr + int(self.m.body_jntnum[link])
                if any(j in controlled for j in range(adr, stop)):
                    moved.add(b)
                    break
                link = int(self.m.body_parentid[link])
        geoms = [
            g
            for g in range(self.m.ngeom)
            if int(self.m.geom_bodyid[g]) in moved
            and (self.m.geom_contype[g] | self.m.geom_conaffinity[g])
        ]
        return np.array(geoms, dtype=int), moved

    def environment(self, body):
        """World-attached or free-floating bodies are obstacles, not robot links."""
        if body == 0:
            return True
        root = body
        while int(self.m.body_parentid[root]) != 0:
            root = int(self.m.body_parentid[root])
        adr = int(self.m.body_jntadr[root])
        stop = adr + int(self.m.body_jntnum[root])
        return any(
            self.m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in range(adr, stop)
        )

    def obstacles(self):
        """Live collidable environment geoms, minus whatever the hands are holding."""
        live = (
            self.m.geom_contype[self.env_geoms]
            | self.m.geom_conaffinity[self.env_geoms]
        ) != 0
        if self.ignore_bodies:
            live &= ~np.isin(self.env_bodies, list(self.ignore_bodies))
        return self.env_geoms[live]

    def candidates(self, obstacles):
        """Bounding-sphere prefilter; exact distances are far too costly per pair."""
        if not len(obstacles) or not len(self.moving_geoms):
            return np.empty((0, 2), dtype=int)
        near = self.d.geom_xpos[self.moving_geoms]
        far = self.d.geom_xpos[obstacles]
        gap = (
            np.linalg.norm(near[:, None, :] - far[None, :, :], axis=-1)
            - self.m.geom_rbound[self.moving_geoms][:, None]
            - self.m.geom_rbound[obstacles][None, :]
        )
        hit = np.argwhere(gap < BROADPHASE)
        if len(hit) > AVOID_PAIRS:
            near = np.argsort(gap[hit[:, 0], hit[:, 1]])[:AVOID_PAIRS]
            hit = hit[near]
        return np.column_stack(
            (self.moving_geoms[hit[:, 0]], obstacles[hit[:, 1]])
        )

    def secondary(self, pairs):
        """Posture pull plus obstacle repulsion, and the closest distance seen."""
        d = self.d
        bias = POSTURE_GAIN * (self.rest[self.q] - d.qpos[self.q])
        nearest = np.inf
        for g1, g2 in pairs:
            dist = mujoco.mj_geomDistance(
                self.m, d, int(g1), int(g2), AVOID_ACTIVATE, self.fromto
            )
            if dist >= AVOID_ACTIVATE:
                continue
            nearest = min(nearest, dist)
            delta = self.fromto[3:] - self.fromto[:3]
            span = np.linalg.norm(delta)
            if span < 1e-9:
                continue
            mujoco.mj_jac(
                self.m,
                d,
                self.jp,
                self.jr,
                self.fromto[:3],
                int(self.m.geom_bodyid[int(g1)]),
            )
            # d(distance)/dq for the witness point, so ascending it separates the pair.
            gradient = -(self.jp[:, self.v].T @ (delta / span))
            weight = np.clip(
                (AVOID_ACTIVATE - dist) / (AVOID_ACTIVATE - AVOID_SAFE), 0.0, 1.0
            )
            bias = bias + AVOID_GAIN * weight * gradient
        urgency = np.clip((AVOID_REQUIRE - nearest) / AVOID_REQUIRE, 0.0, 1.0)
        step = AVOID_STEP + (AVOID_RUSH - AVOID_STEP) * urgency
        bias *= min(1.0, step / max(np.max(np.abs(bias)), 1e-9))
        return bias, nearest

    def solve(self, seed, center, yaw, span, maxiter=180):
        """Place both hooks symmetrically about `center`, `span` apart in hand Y."""
        d = self.d
        m = self.m
        d.qpos[:] = seed
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        targets = [
            np.array(center) + R @ np.array([0, sign * span, GRIP_Z])
            for sign in [1, -1]
        ]
        obstacles = self.obstacles()
        pairs = np.empty((0, 2), dtype=int)
        self.clearance = np.inf
        for step in range(maxiter):
            mujoco.mj_forward(m, d)
            errors = []
            rows = []
            for k, sid in enumerate(self.sites):
                actual = d.site_xmat[sid].reshape(3, 3)
                desired = R @ self.rot[k]
                rotation_delta = desired @ actual.T
                quat = np.empty(4)
                er = np.empty(3)
                mujoco.mju_mat2Quat(quat, rotation_delta.ravel())
                mujoco.mju_quat2Vel(er, quat, 1.0)
                errors.extend(targets[k] - d.site_xpos[sid])
                errors.extend(0.35 * er)
                jp = np.zeros((3, m.nv))
                jr = jp.copy()
                mujoco.mj_jacSite(m, d, jp, jr, sid)
                rows.extend(jp[:, self.v])
                rows.extend(0.35 * jr[:, self.v])
            e = np.array(errors)
            J = np.array(rows)
            if step % AVOID_REFRESH == 0:
                pairs = self.candidates(obstacles)
            bias, self.clearance = self.secondary(pairs)
            if np.max(np.abs(e)) < 0.0003 and self.clearance >= AVOID_REQUIRE:
                return d.qpos.copy(), float(np.max(np.abs(e)))
            free = np.ones(len(self.q), dtype=bool)
            dq = np.zeros(len(self.q))
            for _ in range(4):
                active = J[:, free]
                tilted = bias[free]
                dq[:] = 0
                dq[free] = tilted + active.T @ np.linalg.solve(
                    active @ active.T + np.eye(12) * 0.0001, e - active @ tilted
                )
                at_low = d.qpos[self.q] <= self.limits[:, 0] + 0.002
                at_high = d.qpos[self.q] >= self.limits[:, 1] - 0.002
                blocked = ((at_low & (dq < 0)) | (at_high & (dq > 0))) & free
                if not np.any(blocked):
                    break
                free[blocked] = False
            dq *= min(1.0, 0.08 / max(np.max(np.abs(dq)), 1e-9))
            d.qpos[self.q] = np.clip(
                d.qpos[self.q] + dq,
                self.limits[:, 0] + 0.001,
                self.limits[:, 1] - 0.001,
            )
        return d.qpos.copy(), float(np.max(np.abs(e)))
