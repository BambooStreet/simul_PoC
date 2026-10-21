"""Step 1 pass criterion: wheel contact-point slip over a real in-place turn.

`rolling_wheels()` checks itself analytically at startup. This script is the
empirical counterpart: it drives actual 180 deg turns and reads the contact-point
velocity that falls out of the prescribed qpos stream, for the shipped behaviour
(steering pinned at zero) and for the steered one. Under pure rolling the contact
point is the instantaneous centre of rotation, so every component should be ~0.

    MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/verify_steering.py
"""
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from poc_sim.layout import CARRY  # noqa: E402
from poc_sim.simulation import Simulation  # noqa: E402

WHEELS = [("L", "L_wheel_j2"), ("R", "R_wheel_j2"), ("B", "B_wheel_j2")]
WARMUP = 4  # the first steps cross from the rest pose into the carry pose


def unsteered(sim):
    """Reproduce the pre-step-1 behaviour: steering pinned at 0, rear caster idle."""
    jac = np.zeros((3, sim.m.nv))
    sim.steering = [(adr, 0.0) for adr, _ in sim.steering]
    for adr, _ in sim.steering:
        sim.qrobot[adr] = 0.0
    sim.d.qpos[: sim.nr] = sim.qrobot[: sim.nr]
    mujoco.mj_forward(sim.m, sim.d)
    wheels = []
    for roll in ["L_wheel_j2", "R_wheel_j2"]:
        joint = sim.m.joint(roll)
        body = int(sim.m.jnt_bodyid[joint.id])
        geom = int(np.flatnonzero(sim.m.geom_bodyid == body)[0])
        ground = sim.d.geom_xpos[geom] * [1, 1, 0]
        mujoco.mj_jac(sim.m, sim.d, jac, None, ground, body)
        spin = jac[:, int(joint.dofadr[0])]
        drift = np.cross([0, 0, 1.0], ground - sim.d.body("base").xpos)
        wheels.append((int(joint.qposadr[0]),
                       -float(spin @ drift) / float(spin @ spin)))
    sim.wheels = wheels


def measure(revert):
    sim = Simulation(1)
    if revert:
        unsteered(sim)
    m, d = sim.m, sim.d
    jac = np.zeros((3, m.nv))
    probes = []
    for tag, roll in WHEELS:
        joint = m.joint(roll)
        body = int(m.jnt_bodyid[joint.id])
        probes.append((tag, body, int(np.flatnonzero(m.geom_bodyid == body)[0]),
                       int(joint.dofadr[0])))
    worst = {tag: np.zeros(3) for tag, _ in WHEELS}
    state = {"previous": sim.qrobot.copy(), "n": 0}
    qvel = np.zeros(m.nv)
    inner = sim.step

    def step(q):
        q = np.asarray(q)
        qvel[: sim.nvrobot] = (
            q[: sim.nr] - state["previous"][: sim.nr]
        ) / m.opt.timestep
        state["previous"][: sim.nr] = q[: sim.nr]
        inner(q)
        state["n"] += 1
        if state["n"] <= WARMUP:
            return
        for tag, body, geom, dof in probes:
            ground = d.geom_xpos[geom] * [1, 1, 0]
            mujoco.mj_jac(m, d, jac, None, ground, body)
            heading = jac[:, dof] * [1, 1, 0]
            heading /= np.linalg.norm(heading)
            velocity = jac[:, : sim.nvrobot] @ qvel[: sim.nvrobot]
            across = np.cross([0, 0, 1.0], heading)
            worst[tag] = np.maximum(worst[tag], np.abs([
                velocity @ across, velocity @ heading, velocity[2]]))

    sim.step = step
    sim.turn(np.pi, CARRY)
    sim.turn(0.0, CARRY)
    return worst, state["n"] - WARMUP


for label, revert in [("UNSTEERED (steering pinned at 0)", True),
                      ("STEERED (current code)", False)]:
    worst, n = measure(revert)
    print(f"\n{label} — {n} samples over two 180 deg turns")
    print(f"{'wheel':6s} {'lateral slip':>14s} {'fore-aft':>12s} {'vertical':>12s}")
    for tag, _ in WHEELS:
        lateral, along, vertical = worst[tag]
        print(f"{tag:6s} {lateral:14.3e} {along:12.3e} {vertical:12.3e}")
print("\nunits m/s. lateral slip is the step-1 pass criterion.")
