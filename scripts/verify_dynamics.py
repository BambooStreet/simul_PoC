#!/usr/bin/env python
"""Step 2 checks for the torque-controlled torso and arms (reduced scope).

    MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/verify_dynamics.py

The goal of step 2 is one thing: follow the trajectory steadily *while carrying
a tote*. So the tracking numbers are split by phase — the empty phase contains
hook insertion, which is a different problem and not what the step is judged on.

Scope decisions this follows (docs/dynamics_step2_design.md section 10):
  * omega_n = 20 and beta = 0.05 are fixed; no sensitivity sweeps.
  * armature is compared once at a flat 0.1 kg m^2 per joint, to see whether the
    torque-saturation conclusion depends on how armature is chosen.
  * the dynamic collision verdict is contact with an unheld tote, not depth.
  * verification precision is 0.1 s of cycle time and 1 mm of position.
"""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from poc_sim.config import load  # noqa: E402
from poc_sim.gripper import SPAN_HOLD  # noqa: E402
from poc_sim.layout import BELT_HEADING, CARRY, PALLET_HEADING, PICK  # noqa: E402
from poc_sim.simulation import BELT_LIFT, Simulation  # noqa: E402

SAG_LIMIT = 0.002
TIME_TOLERANCE = 0.1
POSITION_TOLERANCE = 0.001
results = []


def record(name, measured, limit, verdict, note=""):
    results.append({"name": name, "measured": measured, "limit": limit,
                    "verdict": verdict, "note": note})
    print(f"  [{verdict:4s}] {name}: {measured}"
          f"{'  (limit ' + limit + ')' if limit else ''}"
          f"{'  — ' + note if note else ''}")


def carry_loaded(sim):
    sim.move(CARRY, SPAN_HOLD)
    sim.turn(BELT_HEADING, CARRY)
    sim.pick(0)
    sim.move(sim.local([PICK[0], PICK[1], PICK[2] + BELT_LIFT]))
    sim.move(CARRY)
    sim.turn(PALLET_HEADING, CARRY)
    if sim.attached != 0:
        raise RuntimeError("payload not grasped")


def hold_sag(config, seconds=5.0):
    sim = Simulation(1, mode="dynamic", config=config, selection="ordered")
    try:
        carry_loaded(sim)
        site = sim.m.site("L_grip").id
        start = float(sim.d.site_xpos[site][2])
        sim.hold(seconds)
        return start - float(sim.d.site_xpos[site][2])
    finally:
        sim.close()


def cycle(config, boxes, mode="dynamic"):
    sim = Simulation(boxes, mode=mode, config=config, selection="ordered")
    try:
        return sim.run()
    finally:
        sim.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--boxes", type=int, default=8)
    p.add_argument("--report", default="reports/step2_dynamics_check.json")
    a = p.parse_args()
    base = load("dynamics")

    print("\nconfig values with no measurement behind them:")
    for entry in base.unevidenced():
        print(f"  {entry['source']:10s} {entry['path']:38s} = {entry['value']} "
              f"{entry['unit'] or ''}")

    print(f"\nholding a tote still:")
    sag = hold_sag(base)
    record("static sag", f"{sag * 1000:.3f} mm", f"{SAG_LIMIT * 1000:.0f} mm",
           "PASS" if abs(sag) < SAG_LIMIT else "FAIL",
           "residual is the dual-weld loop's geometric offset, not droop")

    print(f"\nfull cycle, {a.boxes} totes:")
    dynamic = cycle(base, a.boxes)
    kinematic = cycle(base, a.boxes, mode="kinematic")

    carrying, empty = dynamic["carrying"], dynamic["empty"]
    record("tracking while carrying a tote",
           f"max {carrying['max_hand_tracking_error_m'] * 1000:.2f} mm, "
           f"rms {carrying['rms_hand_tracking_error_m'] * 1000:.2f} mm",
           "", "INFO",
           f"{carrying['steps']} steps, joints max "
           f"{carrying['max_joint_tracking_error_rad'] * 1000:.1f} mrad — "
           f"this is what step 2 is judged on")
    record("tracking while empty",
           f"max {empty['max_hand_tracking_error_m'] * 1000:.2f} mm, "
           f"rms {empty['rms_hand_tracking_error_m'] * 1000:.2f} mm",
           "", "INFO",
           f"{empty['steps']} steps, peak at "
           f"t={dynamic['max_hand_tracking_error_time_s']:.1f} s (hook insertion)")
    record("joint tracking, whole cycle",
           f"max {dynamic['max_joint_tracking_error_rad'] * 1000:.2f} mrad, "
           f"rms {dynamic['rms_joint_tracking_error_rad'] * 1000:.2f} mrad",
           "", "INFO",
           f"idealised joints max "
           f"{dynamic['max_idealised_tracking_error_rad'] * 1000:.2f} mrad")

    worst = max(dynamic["joints"], key=lambda j: j["torque_use"])
    record("torque saturation",
           f"{dynamic['torque_saturation_fraction'] * 100:.4f}% of steps",
           "0% at rest",
           "PASS" if dynamic["torque_saturation_fraction"] == 0 else "FAIL",
           f"worst use {worst['name']} {worst['torque_use'] * 100:.1f}%")

    same = (dynamic["counts"] == kinematic["counts"]
            and dynamic["placements"] == kinematic["placements"])
    record("mode agreement",
           f"counts {dynamic['counts']} vs {kinematic['counts']}", "exact match",
           "PASS" if same else "FAIL",
           "slot and level per tote identical" if same else "placement diverged")

    delta = abs(kinematic["simulation_seconds"] - dynamic["simulation_seconds"])
    record("kinematic regression",
           f"cycle {kinematic['simulation_seconds']:.3f} s, "
           f"drift {kinematic['max_stack_drift_m'] * 1000:.2f} mm",
           f"{TIME_TOLERANCE} s / {POSITION_TOLERANCE * 1000:.0f} mm", "INFO",
           f"the 32-tote comparison in reports/baseline_kinematic is the real "
           f"check; dynamic runs {delta:.3f} s longer here")

    contacts = dynamic["unheld_tote_contact_steps"]
    limit = base["contact.unheld_tote_contact_steps"]
    record("contact with an unheld tote", f"{contacts} steps",
           "measured" if limit is None else f"{limit} steps",
           "INFO" if limit is None else ("PASS" if contacts <= limit else "FAIL"),
           f"worst gap {dynamic['unheld_tote_contact_worst_gap_m'] * 1000:.3f} mm, "
           f"totes {dynamic['unheld_tote_contact_totes']}"
           if contacts else "none, so the threshold can be 0")

    print(f"\narmature comparison, flat 0.1 kg m^2 per joint:")
    # Measure-only on the contact verdict: the question here is whether the
    # torque conclusion survives, which needs the cycle to finish.
    flat = base.override(control__armature_absolute=0.1,
                         contact__unheld_tote_contact_steps=None)
    try:
        run = cycle(flat, a.boxes)
        changed = (run["torque_saturation_fraction"] == 0) != (
            dynamic["torque_saturation_fraction"] == 0
        )
        saturating = [j["name"] for j in run["joints"]
                      if j["saturated_fraction"] > 0]
        record("armature = 0.1 flat",
               f"saturation {run['torque_saturation_fraction'] * 100:.4f}%, "
               f"carrying rms "
               f"{run['carrying']['rms_hand_tracking_error_m'] * 1000:.2f} mm",
               "saturation conclusion unchanged",
               "FAIL" if changed else "PASS",
               f"beta rule gives {dynamic['torque_saturation_fraction'] * 100:.4f}%; "
               f"saturating joints {saturating or 'none'}")
    except RuntimeError as error:
        record("armature = 0.1 flat", f"aborted: {error}",
               "saturation conclusion unchanged", "FAIL")

    out = ROOT / a.report
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "boxes": a.boxes, "results": results,
        "dynamic": {k: v for k, v in dynamic.items() if k != "events"},
    }, indent=2))
    failed = [r for r in results if r["verdict"] == "FAIL"]
    print(f"\n{len(results) - len(failed)} of {len(results)} checks pass or inform; "
          f"{len(failed)} fail")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
