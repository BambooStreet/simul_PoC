"""How a commanded joint trajectory becomes motion.

`KinematicDrive` writes the trajectory straight into the state — ideal position
replay, which is what the frozen baseline in `reports/baseline_kinematic/` was
recorded with. `DynamicDrive` puts torque actuators on the 17 torso and arm
joints and tracks the same trajectory through them, so joint torque, saturation
and tracking error become real quantities.

Step 2 is deliberately hybrid: the base yaw, the head and the wheels stay
idealised position constraints (marked `idealised` in the config) and become real
motors in step 4.
"""

import xml.etree.ElementTree as E

import mujoco
import numpy as np

TORQUE_JOINTS = [f"torso_j{i}" for i in range(1, 4)] + [
    f"{side}_arm_j{i}" for side in ["L", "R"] for i in range(1, 8)
]
IDEALISED_JOINTS = (
    ["base_yaw"]
    + [f"head_j{i}" for i in range(1, 4)]
    + [f"{side}_wheel_j{i}" for side in ["L", "R", "B"] for i in (1, 2)]
)


def urdf_efforts(urdf_path):
    root = E.parse(urdf_path).getroot()
    efforts = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and limit.get("effort"):
            efforts[joint.get("name")] = float(limit.get("effort"))
    return efforts


def add_actuators(root, efforts):
    """Torque actuators on the driven joints, stiff position holds on the rest.

    Gains are left at 1 here and overwritten once the mass matrix has been
    measured at the carry pose; only the actuator set is structural.
    """
    actuator = E.SubElement(root, "actuator")
    for name in TORQUE_JOINTS:
        effort = efforts[name]
        E.SubElement(
            actuator, "motor", joint=name, name=f"tau_{name}",
            forcerange=f"{-effort} {effort}", ctrlrange=f"{-effort} {effort}",
        )
    for name in IDEALISED_JOINTS:
        # No forcerange: this is an acknowledged idealisation, not a motor.
        E.SubElement(
            actuator, "position", joint=name, name=f"hold_{name}",
            kp="1", dampratio="1",
        )


class Drive:
    """One physics step's worth of command, and the metrics that fall out."""

    name = "drive"

    def __init__(self, sim):
        self.sim = sim

    def command(self, q_des, v_des):
        """Write the command for the step about to be taken."""
        raise NotImplementedError

    def settle(self, q_des):
        """Reconcile state and accumulate metrics after the step."""

    def metrics(self):
        return {}


class KinematicDrive(Drive):
    """Ideal position replay. Explicitly not torque-controlled or balancing.

    The physics integration still runs — the totes are dynamic — but the robot's
    own state is overwritten, so its integration result is discarded.
    """

    name = "kinematic"

    def command(self, q_des, v_des):
        s = self.sim
        old = s.d.qpos[: s.nr].copy()
        s.d.qpos[: s.nr] = q_des[: s.nr]
        s.d.qvel[: s.nvrobot] = (q_des[: s.nr] - old) / s.m.opt.timestep

    def settle(self, q_des):
        s = self.sim
        s.d.qpos[: s.nr] = q_des[: s.nr]
        s.d.qvel[: s.nvrobot] = 0


class DynamicDrive(Drive):
    """Torque control on 17 joints, idealised position holds on the other 10.

        tau = qfrc_bias            inverse dynamics feedforward (gravity, Coriolis)
            - payload              the carried tote, which hangs off a weld
            + kp (q_des - q)       proportional term
            + kv (v_des - v)       derivative term

    The design sketch split `-kv v` out into joint damping, on the grounds that
    an explicit kv of ~1170 would be unstable. Measured, it is not: at dt = 2 ms
    `kv*dt` is 3.8% of the `2(M + armature)` bound on every joint. Splitting it
    is actively harmful, because `kv*v_des` then passes through the actuator and
    eats the `forcerange` that is supposed to represent the motor — `torso_j2`
    saturated at its 380 N*m limit on a term that physically cancels. Keeping the
    whole PD explicit makes `actuator_force` the real motor torque, which is what
    the saturation metric is for.
    """

    name = "dynamic"

    def __init__(self, sim, config, efforts):
        super().__init__(sim)
        m, d = sim.m, sim.d
        omega = config["control.omega_n"]
        zeta = config["control.zeta"]
        beta = config["control.armature_beta"]
        floor = config["control.armature_floor"]
        absolute = config["control.armature_absolute"]
        dt = m.opt.timestep
        self.hands = config["control.payload_hands"]
        self.explicit_damping = config["control.explicit_damping"]
        self.feedforward_alpha = dt / (config["control.velocity_feedforward_tau"] + dt)

        self.tools = [m.body(f"{side}_tool").id for side in ["L", "R"]]
        # Mass-matrix diagonal at the carry pose. Measured before armature is
        # injected, since MuJoCo folds armature into the same matrix.
        mass = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, mass)
        # A carried tote is welded to the hands, not a child in the kinematic
        # tree, so mj_fullM leaves it out entirely. For the wrist that is the
        # difference between 0.013 and 0.28 kg*m^2 — sizing gains on the bare
        # link inertia leaves those joints 20x too soft to hold a tote through a
        # move. Reflect the payload through the hands and size the gains on the
        # loaded carry pose, which is the worst case the cycle contains.
        mass = mass + self.payload_inertia(mass.shape[0])
        self.joints = []
        for name in TORQUE_JOINTS:
            joint = m.joint(name)
            dof = int(joint.dofadr[0])
            inertia = float(mass[dof, dof])
            kp = inertia * omega**2
            kv = 2 * zeta * inertia * omega
            if not self.explicit_damping:
                m.dof_damping[dof] = kv
            m.dof_armature[dof] = (
                absolute if absolute is not None
                else max(beta * inertia, 2 * kp * dt**2, floor)
            )
            self.joints.append({
                "name": name, "dof": dof, "adr": int(joint.qposadr[0]),
                "actuator": m.actuator(f"tau_{name}").id,
                "inertia": inertia, "kp": kp, "kv": kv,
                "armature": float(m.dof_armature[dof]), "effort": efforts[name],
            })
        ideal_omega = config["idealised.omega_n"]
        self.holds = []
        for name in IDEALISED_JOINTS:
            joint = m.joint(name)
            dof = int(joint.dofadr[0])
            inertia = float(mass[dof, dof])
            actuator = m.actuator(f"hold_{name}").id
            gain = inertia * ideal_omega**2
            m.actuator_gainprm[actuator, 0] = gain
            m.actuator_biasprm[actuator, 1] = -gain
            m.actuator_biasprm[actuator, 2] = -2 * inertia * ideal_omega
            self.holds.append({"adr": int(joint.qposadr[0]), "actuator": actuator})

        self.jacobian = np.zeros((3, m.nv))
        self.grip_jacobian = np.zeros((3, m.nv))
        self.grips = [m.site(f"{side}_grip").id for side in ["L", "R"]]
        self.held = np.array([h["adr"] for h in self.holds])
        self.held_dofs = np.array(
            [int(m.joint(n).dofadr[0]) for n in IDEALISED_JOINTS]
        )
        self.dofs = np.array([j["dof"] for j in self.joints])
        self.adrs = np.array([j["adr"] for j in self.joints])
        self.actuators = np.array([j["actuator"] for j in self.joints])
        self.kps = np.array([j["kp"] for j in self.joints])
        self.kvs = np.array([j["kv"] for j in self.joints])
        self.limits = np.array([j["effort"] for j in self.joints])
        self.worst_joint_error = np.zeros(len(self.joints))
        self.error_squared = np.zeros(len(self.joints))
        self.saturated = np.zeros(len(self.joints))
        self.worst_torque = np.zeros(len(self.joints))
        self.worst_cartesian = 0.0
        self.cartesian_squared = 0.0
        self.worst_held_error = 0.0
        # Step 2's goal is tracking *while carrying a tote*, so keep the loaded
        # and empty phases apart: the empty phase contains the hook insertion,
        # which is a different problem.
        self.carrying = {"worst": 0.0, "squared": 0.0, "steps": 0, "joint": 0.0}
        self.empty = {"worst": 0.0, "squared": 0.0, "steps": 0, "joint": 0.0}
        self.worst_penetration = 0.0
        self.worst_cartesian_time = 0.0
        self.filtered_rate = np.zeros(len(self.joints))
        self.steps = 0

    def payload_inertia(self, nv):
        """Inertia a carried tote reflects into the joints, through both hands.

        Evaluated with a virtual payload at the commanded carry-pose hand centre,
        in the hand's orientation: gains stay constant for the whole cycle, as
        the design calls for, and are sized for the loaded case. Running empty
        then leaves the joints over-damped, which costs tracking nothing.
        """
        s = self.sim
        body = s.m.body("tote_0").id
        mass = float(s.m.body_mass[body])
        rotation = s.d.xmat[self.tools[0]].reshape(3, 3)
        inertia = rotation @ np.diag(s.m.body_inertia[body]) @ rotation.T
        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        total = np.zeros((nv, nv))
        for tool in self.tools:
            mujoco.mj_jac(s.m, s.d, jacp, jacr, s.center, tool)
            total += (mass * jacp.T @ jacp + jacr.T @ inertia @ jacr) / self.hands
        return total

    def payload_torque(self):
        """Generalised force the carried tote's weight exerts on the arms.

        The tote is a free body held by welds, so it is not in the robot's
        kinematic tree and its weight never reaches `qfrc_bias`. It is rigid with
        the hands though, so take the Jacobian of its centre of mass as a point
        on each tool body and share the weight between them.
        """
        s = self.sim
        if s.attached is None:
            return 0.0
        body = s.m.body(f"tote_{s.attached}").id
        weight = float(s.m.body_mass[body]) * s.m.opt.gravity / self.hands
        total = np.zeros(s.m.nv)
        for tool in self.tools:
            mujoco.mj_jac(s.m, s.d, self.jacobian, None, s.d.xipos[body], tool)
            total += self.jacobian.T @ weight
        return total

    def command(self, q_des, v_des):
        s = self.sim
        payload = self.payload_torque()
        bias = s.d.qfrc_bias[self.dofs]
        if not np.isscalar(payload):
            bias = bias - payload[self.dofs]
        error = q_des[self.adrs] - s.d.qpos[self.adrs]
        # The commanded velocity steps at every waypoint, so feed it forward
        # through a one-pole filter; the proportional term picks up the lag.
        target = np.zeros(len(self.joints)) if v_des is None else v_des[self.dofs]
        self.filtered_rate += self.feedforward_alpha * (target - self.filtered_rate)
        rate = self.filtered_rate
        if self.explicit_damping:
            rate = rate - s.d.qvel[self.dofs]
        s.d.ctrl[self.actuators] = bias + self.kps * error + self.kvs * rate
        for hold in self.holds:
            s.d.ctrl[hold["actuator"]] = q_des[hold["adr"]]

    def settle(self, q_des):
        s = self.sim
        error = np.abs(q_des[self.adrs] - s.d.qpos[self.adrs])
        self.worst_joint_error = np.maximum(self.worst_joint_error, error)
        self.error_squared += error**2
        force = np.abs(s.d.actuator_force[self.actuators])
        self.worst_torque = np.maximum(self.worst_torque, force)
        self.saturated += force >= 0.99 * self.limits
        # Cartesian tracking error, first order from the joint error. Comparing
        # the hand against `sim.center` instead would measure how far the current
        # waypoint still is, not how well the robot follows.
        delta = np.zeros(s.m.nv)
        delta[self.dofs] = q_des[self.adrs] - s.d.qpos[self.adrs]
        held = q_des[self.held] - s.d.qpos[self.held]
        delta[self.held_dofs] = held
        self.worst_held_error = max(self.worst_held_error, float(np.abs(held).max()))
        offset = 0.0
        for grip in self.grips:
            mujoco.mj_jacSite(s.m, s.d, self.grip_jacobian, None, grip)
            offset = max(offset, float(np.linalg.norm(self.grip_jacobian @ delta)))
        if offset > self.worst_cartesian:
            self.worst_cartesian = offset
            self.worst_cartesian_time = float(s.d.time)
        self.cartesian_squared += offset**2
        phase = self.carrying if s.attached is not None else self.empty
        phase["worst"] = max(phase["worst"], offset)
        phase["squared"] += offset**2
        phase["joint"] = max(phase["joint"], float(error.max()))
        phase["steps"] += 1
        if s.d.ncon:
            self.worst_penetration = min(
                self.worst_penetration, float(s.d.contact.dist[: s.d.ncon].min())
            )
        self.steps += 1

    @staticmethod
    def phase_metrics(phase):
        steps = max(phase["steps"], 1)
        return {
            "steps": phase["steps"],
            "max_hand_tracking_error_m": phase["worst"],
            "rms_hand_tracking_error_m": float(np.sqrt(phase["squared"] / steps)),
            "max_joint_tracking_error_rad": phase["joint"],
        }

    def metrics(self):
        n = max(self.steps, 1)
        return {
            "max_joint_tracking_error_rad": float(self.worst_joint_error.max()),
            "rms_joint_tracking_error_rad": float(
                np.sqrt(self.error_squared.max() / n)
            ),
            "max_hand_tracking_error_m": self.worst_cartesian,
            "rms_hand_tracking_error_m": float(np.sqrt(self.cartesian_squared / n)),
            "max_idealised_tracking_error_rad": self.worst_held_error,
            "max_hand_tracking_error_time_s": self.worst_cartesian_time,
            "carrying": self.phase_metrics(self.carrying),
            "empty": self.phase_metrics(self.empty),
            "max_contact_penetration_m": -self.worst_penetration,
            "torque_saturation_fraction": float(self.saturated.max() / n),
            "joints": [
                {
                    "name": j["name"],
                    "inertia_kg_m2": j["inertia"],
                    "kp": j["kp"],
                    "kv": j["kv"],
                    "armature": j["armature"],
                    "effort_limit_nm": j["effort"],
                    "max_torque_nm": float(self.worst_torque[i]),
                    "torque_use": float(self.worst_torque[i] / j["effort"]),
                    "saturated_fraction": float(self.saturated[i] / n),
                    "max_tracking_error_rad": float(self.worst_joint_error[i]),
                }
                for i, j in enumerate(self.joints)
            ],
        }
