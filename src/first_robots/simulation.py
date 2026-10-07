"""Bounded deterministic SO-101 virtual motion; no device or language-model calls."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
ALL_JOINTS = (*ARM_JOINTS, "gripper")


def load_model(model_path: Path) -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if model.nq != 6 or model.nv != 6 or model.nu != 6:
        raise ValueError("The reach baseline requires the six-joint SO-101 model without object DOFs")
    for index, name in enumerate(ALL_JOINTS):
        joint = model.joint(name)
        actuator = model.actuator(name)
        if model.jnt_qposadr[joint.id] != index or model.actuator_trnid[actuator.id, 0] != joint.id:
            raise ValueError("Unexpected SO-101 joint/actuator ordering")
        if not model.jnt_limited[joint.id] or not model.actuator_ctrllimited[actuator.id]:
            raise ValueError("All motion axes must declare joint and actuator limits")
    return model


def command_limits(model: mujoco.MjModel) -> np.ndarray:
    return np.column_stack((np.maximum(model.jnt_range[:, 0], model.actuator_ctrlrange[:, 0]), np.minimum(model.jnt_range[:, 1], model.actuator_ctrlrange[:, 1])))


def penetrating_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict]:
    contacts = []
    for contact in data.contact:
        body1 = model.geom_bodyid[contact.geom1]
        body2 = model.geom_bodyid[contact.geom2]
        # Fixed mounting contacts are not motion collisions.
        if model.body_weldid[body1] == 0 and model.body_weldid[body2] == 0:
            continue
        if contact.dist < 0:
            contacts.append({"geom1": model.geom(contact.geom1).name, "geom2": model.geom(contact.geom2).name, "distance_m": float(contact.dist)})
    return contacts


def solve_position(model: mujoco.MjModel, site_name: str, target_world_m: list[float], initial_qpos_rad: list[float], settings: dict) -> dict:
    target = np.asarray(target_world_m, dtype=float)
    qpos = np.asarray(initial_qpos_rad, dtype=float).copy()
    limits = command_limits(model)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError("Target must be three finite world coordinates")
    if qpos.shape != (6,) or not np.isfinite(qpos).all() or np.any(qpos < limits[:, 0]) or np.any(qpos > limits[:, 1]):
        raise ValueError("Initial joints violate finite shape or model/actuator limits")
    if settings["max_iterations"] < 1 or settings["max_iterations"] > 10000 or settings["line_search_steps"] < 1 or settings["line_search_steps"] > 30 or settings["damping"] <= 0 or settings["position_tolerance_m"] <= 0 or settings["max_joint_step_rad"] <= 0:
        raise ValueError("Invalid bounded IK settings")
    data = mujoco.MjData(model)
    site_id = model.site(site_name).id
    jacobian = np.zeros((3, model.nv))
    reason = "iteration_budget_exhausted"
    for iteration in range(settings["max_iterations"]):
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        error = target - data.site_xpos[site_id]
        if np.linalg.norm(error) <= settings["position_tolerance_m"]:
            reason = "position_solver_converged"
            break
        mujoco.mj_jacSite(model, data, jacobian, None, site_id)
        arm_jacobian = jacobian[:, :5]
        increment = arm_jacobian.T @ np.linalg.solve(arm_jacobian @ arm_jacobian.T + settings["damping"] ** 2 * np.eye(3), error)
        scale = min(1.0, settings["max_joint_step_rad"] / max(float(np.max(np.abs(increment))), 1e-15))
        accepted = False
        for trial in range(settings["line_search_steps"]):
            candidate = qpos.copy()
            candidate[:5] = np.clip(qpos[:5] + increment * scale * 0.5 ** trial, limits[:5, 0], limits[:5, 1])
            data.qpos[:] = candidate
            mujoco.mj_forward(model, data)
            if np.linalg.norm(target - data.site_xpos[site_id]) < np.linalg.norm(error):
                qpos = candidate
                accepted = True
                break
        if not accepted:
            reason = "no_improving_limited_step"
            break
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    residual = float(np.linalg.norm(target - data.site_xpos[site_id]))
    if residual <= settings["position_tolerance_m"]:
        reason = "position_solver_converged"
    return {"converged": bool(residual <= settings["position_tolerance_m"]), "reason": reason, "iterations": iteration + 1, "qpos_rad": qpos.tolist(), "site_world_m": data.site_xpos[site_id].tolist(), "position_residual_m": residual}


def simulate_reach(model_path: Path, configuration: dict) -> tuple[dict, dict[str, np.ndarray]]:
    model = load_model(model_path)
    start = np.asarray(configuration["initial_qpos_rad"], dtype=float)
    target = np.asarray(configuration["target_world_m"], dtype=float)
    solution = solve_position(model, configuration["site"], target.tolist(), start.tolist(), configuration["ik"])
    report = {"ik": solution, "execution_applied": False, "reason": solution["reason"], "scope": "fixed virtual position-only motion; no grasp orientation or object transport"}
    if not solution["converged"]:
        return report, {}
    goal = np.asarray(solution["qpos_rad"])
    motion = configuration["motion"]
    if not (0 < motion["duration_s"] <= 60 and 0 <= motion["settle_s"] <= 60 and 2 <= motion["path_check_samples"] <= 10000):
        raise ValueError("Invalid bounded motion settings")
    scratch = mujoco.MjData(model)
    for fraction in np.linspace(0, 1, motion["path_check_samples"]):
        scratch.qpos[:] = start + fraction * (goal - start)
        mujoco.mj_forward(model, scratch)
        collisions = penetrating_contacts(model, scratch)
        if collisions:
            report.update(reason="sampled_path_penetration", blocking_contacts=collisions, path_fraction=float(fraction))
            return report, {}
    data = mujoco.MjData(model)
    data.qpos[:] = start
    data.ctrl[:] = start
    mujoco.mj_forward(model, data)
    site_id = model.site(configuration["site"]).id
    initial_error = float(np.linalg.norm(data.site_xpos[site_id] - target))
    limits = command_limits(model)
    steps = int(np.ceil((motion["duration_s"] + motion["settle_s"]) / model.opt.timestep))
    trace = {name: [] for name in ("time_s", "qpos_rad", "qvel_rad_s", "ctrl_rad", "site_world_m", "position_error_m")}
    reason = "motion_completed"
    for step in range(steps + 1):
        mujoco.mj_forward(model, data)
        for name, value in (("time_s", float(data.time)), ("qpos_rad", data.qpos.copy()), ("qvel_rad_s", data.qvel.copy()), ("ctrl_rad", data.ctrl.copy()), ("site_world_m", data.site_xpos[site_id].copy()), ("position_error_m", float(np.linalg.norm(data.site_xpos[site_id] - target)))):
            trace[name].append(value)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            reason = "nonfinite_simulation_state"
            break
        if np.any(data.qpos < limits[:, 0]) or np.any(data.qpos > limits[:, 1]):
            reason = "actual_joint_limit_violation"
            break
        collisions = penetrating_contacts(model, data)
        if collisions:
            report["blocking_contacts"] = collisions
            reason = "actual_path_penetration"
            break
        if step == steps:
            break
        fraction = min(1.0, float(data.time) / motion["duration_s"])
        smooth_fraction = fraction ** 2 * (3 - 2 * fraction)
        data.ctrl[:] = start + smooth_fraction * (goal - start)
        mujoco.mj_step(model, data)
    trace_arrays = {name: np.asarray(values) for name, values in trace.items()}
    report.update(
        execution_applied=True,
        reason=reason,
        simulated_steps=len(trace["time_s"]) - 1,
        initial_position_error_m=initial_error,
        final_position_error_m=float(trace_arrays["position_error_m"][-1]),
        final_site_world_m=trace_arrays["site_world_m"][-1].tolist(),
        actual_joint_limits_satisfied=bool(np.all(trace_arrays["qpos_rad"] >= limits[:, 0]) and np.all(trace_arrays["qpos_rad"] <= limits[:, 1])),
        position_error_reduced=bool(trace_arrays["position_error_m"][-1] < initial_error),
        path_check_samples=motion["path_check_samples"],
        finite_trajectory=bool(all(np.isfinite(values).all() for values in trace_arrays.values())),
    )
    return report, trace_arrays


def render_reach_replay(model_path: Path, configuration: dict, trace: dict[str, np.ndarray], *, frame_stride: int = 10) -> list[np.ndarray]:
    spec = mujoco.MjSpec.from_file(str(model_path))
    marker = spec.worldbody.add_geom()
    marker.name = "reach_target_marker"
    marker.type = mujoco.mjtGeom.mjGEOM_SPHERE
    marker.pos = configuration["target_world_m"]
    marker.size = [0.008, 0, 0]
    marker.rgba = [0.1, 0.9, 0.3, 0.65]
    marker.contype = marker.conaffinity = 0
    model = spec.compile()
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.15, 0, 0.15]
    camera.distance = 0.8
    camera.azimuth = 135
    camera.elevation = -25
    frames = []
    indices = list(range(0, len(trace["qpos_rad"]), frame_stride))
    if indices[-1] != len(trace["qpos_rad"]) - 1:
        indices.append(len(trace["qpos_rad"]) - 1)
    with mujoco.Renderer(model, height=360, width=480) as renderer:
        for index in indices:
            data.qpos[:] = trace["qpos_rad"][index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera)
            frames.append(renderer.render().copy())
    return frames
