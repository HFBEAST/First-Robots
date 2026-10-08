"""Idealized cylinder fixture, bounded side-pose solver and contact-driven task."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .simulation import ALL_JOINTS


def build_cylinder_scene(model_path: Path, configuration: dict, *, scene_spec: mujoco.MjSpec | None = None) -> tuple[mujoco.MjModel, mujoco.MjData]:
    cylinder = configuration["cylinder"]
    dimensions = np.asarray([cylinder["radius_m"], cylinder["half_height_m"], cylinder["mass_kg"]], dtype=float)
    center = np.asarray(cylinder["initial_center_world_m"], dtype=float)
    if not np.isfinite(dimensions).all() or np.any(dimensions <= 0) or center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("Cylinder must have finite positive dimensions/mass and a finite center")
    spec = scene_spec if scene_spec is not None else mujoco.MjSpec.from_file(str(model_path))
    body = spec.worldbody.add_body()
    body.name = "task_cylinder"
    body.pos = cylinder["initial_center_world_m"]
    joint = body.add_joint()
    joint.name = "cylinder_free"
    joint.type = mujoco.mjtJoint.mjJNT_FREE
    shape = body.add_geom()
    shape.name = "task_cylinder_geom"
    shape.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    shape.size = [cylinder["radius_m"], cylinder["half_height_m"], 0]
    shape.mass = cylinder["mass_kg"]
    shape.rgba = [0.9, 0.15, 0.5, 1]
    shape.friction = [1, 0.005, 0.0001]
    shape.condim = 6
    model = spec.compile()
    limits = robot_limits(model)
    initial = np.asarray(configuration["initial_qpos_rad"], dtype=float)
    if initial.shape != (6,) or not np.isfinite(initial).all() or np.any(initial < limits[:, 0]) or np.any(initial > limits[:, 1]):
        raise ValueError("Initial fixture joints violate model or actuator limits")
    data = mujoco.MjData(model)
    data.qpos[:6] = configuration["initial_qpos_rad"]
    data.ctrl[:] = configuration["initial_qpos_rad"]
    mujoco.mj_forward(model, data)
    return model, data


def robot_limits(model: mujoco.MjModel) -> np.ndarray:
    joint_ids = [model.joint(name).id for name in ALL_JOINTS]
    actuator_ids = [model.actuator(name).id for name in ALL_JOINTS]
    return np.column_stack((np.maximum(model.jnt_range[joint_ids, 0], model.actuator_ctrlrange[actuator_ids, 0]), np.minimum(model.jnt_range[joint_ids, 1], model.actuator_ctrlrange[actuator_ids, 1])))


def solve_side_pose(model: mujoco.MjModel, target_world_m: list[float], initial_qpos_rad: list[float], settings: dict) -> dict:
    """Fix wrist roll; solve XYZ and horizontal approach, without a six-DOF demand."""
    target = np.asarray(target_world_m, dtype=float)
    qpos = np.asarray(initial_qpos_rad, dtype=float).copy()
    limits = robot_limits(model)
    if target.shape != (3,) or not np.isfinite(target).all() or qpos.shape != (6,) or not np.isfinite(qpos).all():
        raise ValueError("Side pose requires finite target and six initial robot joints")
    if not (1 <= settings["max_iterations"] <= 10000 and settings["damping"] > 0 and settings["orientation_weight_m"] > 0 and settings["numerical_tolerance"] > 0 and settings["max_joint_step_rad"] > 0):
        raise ValueError("Invalid bounded side-pose numerical settings")
    qpos[4] = settings["wrist_roll_rad"]
    if np.any(qpos < limits[:, 0]) or np.any(qpos > limits[:, 1]):
        raise ValueError("Side-pose initial state violates model or actuator limits")
    data = mujoco.MjData(model)
    site_id = model.site("gripperframe").id
    position_jacobian = np.zeros((3, model.nv))
    rotation_jacobian = np.zeros((3, model.nv))
    weight = settings["orientation_weight_m"]

    def residual(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        approach = data.site_xmat[site_id].reshape(3, 3)[:, 0]
        return np.r_[target - data.site_xpos[site_id], -weight * approach[2]], approach

    reason = "iteration_budget_exhausted"
    for iteration in range(settings["max_iterations"]):
        error, approach = residual(qpos)
        if np.linalg.norm(error) <= settings["numerical_tolerance"]:
            reason = "side_pose_converged"
            break
        mujoco.mj_jacSite(model, data, position_jacobian, rotation_jacobian, site_id)
        orientation_row = weight * (np.cross(approach, [0, 0, 1]) @ rotation_jacobian[:, :4])
        jacobian = np.vstack((position_jacobian[:, :4], orientation_row))
        step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + settings["damping"] ** 2 * np.eye(4), error)
        scale = min(1.0, settings["max_joint_step_rad"] / max(float(np.max(np.abs(step))), 1e-15))
        improved = False
        for trial in range(10):
            candidate = qpos.copy()
            candidate[:4] = np.clip(qpos[:4] + step * scale * 0.5 ** trial, limits[:4, 0], limits[:4, 1])
            candidate_error, _ = residual(candidate)
            if np.linalg.norm(candidate_error) < np.linalg.norm(error):
                qpos = candidate
                improved = True
                break
        if not improved:
            reason = "no_improving_limited_step"
            break
    error, approach = residual(qpos)
    converged = bool(np.linalg.norm(error) <= settings["numerical_tolerance"])
    if converged:
        reason = "side_pose_converged"
    return {"converged": converged, "reason": reason, "iterations": iteration + 1, "qpos_rad": qpos.tolist(), "position_residual_m": float(np.linalg.norm(error[:3])), "approach_world_axis": approach.tolist(), "weighted_pose_residual_m": float(np.linalg.norm(error)), "site_world_m": data.site_xpos[site_id].tolist()}


def cylinder_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    cylinder_geom = model.geom("task_cylinder_geom").id
    floor_geom = model.geom("floor").id
    jaw_bodies = {model.body("gripper").id: "fixed_jaw_force_n", model.body("moving_jaw_so101_v1").id: "moving_jaw_force_n"}
    forces = {"fixed_jaw_force_n": 0.0, "moving_jaw_force_n": 0.0, "floor_support_force_n": 0.0}
    forbidden = []
    for index, contact in enumerate(data.contact):
        pair = {contact.geom1, contact.geom2}
        body1, body2 = model.geom_bodyid[contact.geom1], model.geom_bodyid[contact.geom2]
        if model.body_weldid[body1] == 0 and model.body_weldid[body2] == 0:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        allowed = False
        if cylinder_geom in pair:
            other = contact.geom2 if contact.geom1 == cylinder_geom else contact.geom1
            other_name = model.geom(other).name or ""
            other_body = model.geom_bodyid[other]
            if other == floor_geom:
                forces["floor_support_force_n"] += float(force[0])
                allowed = True
            elif other_body in jaw_bodies and (other_name.startswith(("fixed_jaw_", "moving_jaw_")) or model.geom_group[other] == 4):
                forces[jaw_bodies[other_body]] += float(force[0])
                allowed = True
        if not allowed and contact.dist < 0:
            forbidden.append({"geom1": model.geom(contact.geom1).name, "geom2": model.geom(contact.geom2).name, "geom1_id": int(contact.geom1), "geom2_id": int(contact.geom2), "body1": model.body(body1).name, "body2": model.body(body2).name, "distance_m": float(contact.dist)})
    return {**forces, "forbidden_penetrations": forbidden}


def simulate_pick_place(model_path: Path, configuration: dict) -> tuple[dict, dict[str, np.ndarray]]:
    """Execute gated physical phases; object state is never assigned after initialization."""
    model, data = build_cylinder_scene(model_path, configuration)
    limits = robot_limits(model)
    motion = configuration["motion"]
    region = configuration["placement_region"]
    region_center = np.asarray(region["center_world_m"], dtype=float)
    if region_center.shape != (3,) or not np.isfinite(region_center).all() or not 0 < region["radius_m"] < float("inf"):
        raise ValueError("Placement region must have a finite center and positive finite radius")
    if not 0 < configuration["fixture_settle_s"] <= 60:
        raise ValueError("Invalid bounded fixture-settle duration")
    for name in ("move_duration_s", "jaw_duration_s", "hold_s", "final_settle_s"):
        if not 0 < motion[name] <= 60:
            raise ValueError("Invalid bounded phase duration")
    if not 2 <= motion["path_check_samples"] <= 10000:
        raise ValueError("Invalid bounded path-check budget")
    closed = configuration["closed_gripper_command_rad"]
    if not limits[5, 0] <= closed <= limits[5, 1]:
        raise ValueError("Closed gripper command violates model/actuator limits")
    body_id = model.body("task_cylinder").id
    site_id = model.site("gripperframe").id
    trace = {name: [] for name in ("time_s", "qpos", "qvel", "ctrl", "cylinder_center_m", "site_world_m", "phase", "fixed_jaw_force_n", "moving_jaw_force_n", "floor_support_force_n")}
    checkpoints = []
    solutions = {}

    def capture(phase: str) -> str | None:
        mujoco.mj_forward(model, data)
        contacts = cylinder_contacts(model, data)
        values = {"time_s": float(data.time), "qpos": data.qpos.copy(), "qvel": data.qvel.copy(), "ctrl": data.ctrl.copy(), "cylinder_center_m": data.xpos[body_id].copy(), "site_world_m": data.site_xpos[site_id].copy(), "phase": phase, **{name: contacts[name] for name in ("fixed_jaw_force_n", "moving_jaw_force_n", "floor_support_force_n")}}
        for name, value in values.items():
            trace[name].append(value)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or not np.isfinite(data.ctrl).all():
            return "nonfinite_simulation_state"
        if any(warning.number > 0 for warning in data.warning):
            return "simulator_warning"
        if np.any(data.qpos[:6] < limits[:, 0]) or np.any(data.qpos[:6] > limits[:, 1]):
            return "actual_joint_limit_violation"
        if contacts["forbidden_penetrations"]:
            checkpoints.append({"phase": phase, "blocking_contacts": contacts["forbidden_penetrations"]})
            return "forbidden_actual_penetration"
        return None

    def execute_phase(phase: str, goal: np.ndarray, duration: float, hold: float) -> str | None:
        error = capture(phase)
        if error:
            return error
        start = data.qpos[:6].copy()
        scratch = mujoco.MjData(model)
        for fraction in np.linspace(0, 1, motion["path_check_samples"]):
            scratch.qpos[:] = data.qpos
            scratch.qpos[:6] = start + fraction * (goal - start)
            mujoco.mj_forward(model, scratch)
            blocked = cylinder_contacts(model, scratch)["forbidden_penetrations"]
            if blocked:
                checkpoints.append({"phase": phase, "path_fraction": float(fraction), "blocking_contacts": blocked})
                return "forbidden_sampled_penetration"
        steps = int(np.ceil((duration + hold) / model.opt.timestep))
        for step in range(steps + 1):
            if step > 0:
                error = capture(phase)
                if error:
                    return error
            if step == steps:
                break
            fraction = min(1.0, step * model.opt.timestep / duration)
            smooth = fraction ** 2 * (3 - 2 * fraction)
            data.ctrl[:] = start + smooth * (goal - start)
            mujoco.mj_step(model, data)
        contacts = cylinder_contacts(model, data)
        axis = data.xmat[body_id].reshape(3, 3)[:, 2]
        cylinder = configuration["cylinder"]
        bottom_z = float(data.xpos[body_id, 2] - cylinder["half_height_m"] * abs(axis[2]) - cylinder["radius_m"] * np.sqrt(max(0, 1 - axis[2] ** 2)))
        checkpoints.append({"phase": phase, "cylinder_center_world_m": data.xpos[body_id].tolist(), "cylinder_bottom_z_m": bottom_z, "contacts": contacts})
        return None

    reason = "pick_place_completed"
    error = execute_phase("fixture_settle", data.qpos[:6].copy(), configuration["fixture_settle_s"], motion["hold_s"])
    if error:
        reason = error
    phases = (("pregrasp", "pregrasp_site_world_m", False), ("descend", "grasp_site_world_m", False), ("close", None, True), ("lift", "lift_site_world_m", True), ("transfer", "transfer_site_world_m", True), ("lower", "place_site_world_m", True), ("release", None, False), ("retreat", "transfer_site_world_m", False))
    if not error:
        for phase, target_name, gripping in phases:
            goal = data.qpos[:6].copy()
            goal[5] = closed if gripping else configuration["initial_qpos_rad"][5]
            duration = motion["jaw_duration_s"] if target_name is None else motion["move_duration_s"]
            if target_name:
                pose = solve_side_pose(model, configuration[target_name], goal.tolist(), configuration["side_pose_ik"])
                solutions[phase] = pose
                if not pose["converged"]:
                    reason = "phase_ik_failed:" + phase
                    break
                goal = np.asarray(pose["qpos_rad"])
            error = execute_phase(phase, goal, duration, motion["hold_s"])
            if error:
                reason = error + ":" + phase
                break
            last = checkpoints[-1]
            contacts = last["contacts"]
            if phase in ("close", "lift", "transfer") and not (contacts["fixed_jaw_force_n"] > 0 and contacts["moving_jaw_force_n"] > 0):
                reason = "no_opposed_jaw_contact:" + phase
                break
            if phase in ("lift", "transfer") and not (last["cylinder_bottom_z_m"] > 0 and contacts["floor_support_force_n"] == 0):
                reason = "object_not_lifted:" + phase
                break
        if reason == "pick_place_completed":
            error = execute_phase("final_settle", data.qpos[:6].copy(), motion["final_settle_s"], motion["hold_s"])
            if error:
                reason = error
    final_center = data.xpos[body_id].copy()
    radial_error = float(np.linalg.norm(final_center[:2] - region_center[:2]))
    region_contains_cylinder = bool(radial_error + configuration["cylinder"]["radius_m"] <= region["radius_m"])
    contacts = cylinder_contacts(model, data)
    released_on_support = bool(contacts["floor_support_force_n"] > 0 and contacts["fixed_jaw_force_n"] == 0 and contacts["moving_jaw_force_n"] == 0)
    if reason == "pick_place_completed" and not (region_contains_cylinder and released_on_support):
        reason = "placement_postcondition_failed"
    report = {"reason": reason, "completed": reason == "pick_place_completed", "phase_checkpoints": checkpoints, "side_pose_solutions": solutions, "final_cylinder_center_world_m": final_center.tolist(), "evaluation_only_radial_error_to_b_m": radial_error, "region_b_contains_cylinder": region_contains_cylinder, "released_on_support": released_on_support, "final_cylinder_free_joint_velocity": data.qvel[6:].tolist(), "scope": "known-coordinate synthetic cylinder contact task; no camera or natural-language input"}
    return report, {name: np.asarray(values) for name, values in trace.items()}


def render_pick_place_replay(model_path: Path, configuration: dict, trace: dict[str, np.ndarray], *, frame_stride: int = 20) -> list[np.ndarray]:
    model, data = build_cylinder_scene(model_path, configuration)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.2, 0.04, 0.12]
    camera.distance = 0.85
    camera.azimuth = 135
    camera.elevation = -25
    frames = []
    indices = list(range(0, len(trace["qpos"]), frame_stride))
    if indices[-1] != len(trace["qpos"]) - 1:
        indices.append(len(trace["qpos"]) - 1)
    with mujoco.Renderer(model, height=360, width=480) as renderer:
        for index in indices:
            data.qpos[:] = trace["qpos"][index]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera)
            region = configuration["placement_region"]
            marker = renderer.scene.geoms[renderer.scene.ngeom]
            mujoco.mjv_initGeom(marker, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=np.array([region["radius_m"], 0.001, 0]), pos=np.array([*region["center_world_m"][:2], 0.002]), mat=np.eye(3).ravel(), rgba=np.array([0.1, 0.9, 0.3, 0.45]))
            renderer.scene.ngeom += 1
            frames.append(renderer.render().copy())
    return frames
