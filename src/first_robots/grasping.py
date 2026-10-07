"""Idealized single-cylinder fixture and bounded five-axis side-grasp pose solver."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .simulation import ALL_JOINTS


def build_cylinder_scene(model_path: Path, configuration: dict) -> tuple[mujoco.MjModel, mujoco.MjData]:
    cylinder = configuration["cylinder"]
    dimensions = np.asarray([cylinder["radius_m"], cylinder["half_height_m"], cylinder["mass_kg"]], dtype=float)
    center = np.asarray(cylinder["initial_center_world_m"], dtype=float)
    if not np.isfinite(dimensions).all() or np.any(dimensions <= 0) or center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("Cylinder must have finite positive dimensions/mass and a finite center")
    spec = mujoco.MjSpec.from_file(str(model_path))
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
