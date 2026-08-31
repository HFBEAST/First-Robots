"""Run the authorized virtual SO-101 MuJoCo observation preflight.

This research-only runner creates a new formal run directory.  It never reads
product code or controls physical hardware.
"""

from __future__ import annotations

import hashlib
import json
import math
import platform
import shutil
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RUN_ID = "EXP-20260831-003-so101-mujoco-observation-preflight"
ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "research" / "configs" / "EXP-20260831-001-so101-mujoco-observation-preflight.json"
SOURCE_DIR = ROOT / "research" / "sources" / "mujoco_menagerie_robotstudio_so101" / "robotstudio_so101"
SOURCE_SCENE = SOURCE_DIR / "scene.xml"
OUTPUT_DIR = ROOT / "research" / "artifacts" / RUN_ID
RESULT_PATH = ROOT / "research" / "reports" / f"{RUN_ID}.result.json"
RECORD_PATH = ROOT / "research" / "experiments" / f"{RUN_ID}.record.json"


def sha256(path: Path) -> str:
    return f"sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}"


def look_at_quat(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return a MuJoCo camera-frame quaternion looking from position to target."""
    forward = target - position
    forward /= np.linalg.norm(forward)
    camera_z = -forward  # MuJoCo cameras look along their local -Z axis.
    up_hint = np.array([0.0, 0.0, 1.0])
    camera_x = np.cross(up_hint, camera_z)
    camera_x /= np.linalg.norm(camera_x)
    camera_y = np.cross(camera_z, camera_x)
    matrix = np.column_stack((camera_x, camera_y, camera_z)).reshape(9)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, matrix)
    return quat


def estimate_reach(model: mujoco.MjModel) -> float:
    """Conservatively estimate radial gripper-site reach from fixed joint samples."""
    generator = np.random.default_rng(0)
    data = mujoco.MjData(model)
    base_site = model.site("baseframe").id
    gripper_site = model.site("gripperframe").id
    maximum = 0.0
    for qpos in generator.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1], size=(10_000, model.nq)):
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        offset = data.site_xpos[gripper_site] - data.site_xpos[base_site]
        maximum = max(maximum, float(np.linalg.norm(offset[:2])))
    return maximum


def add_virtual_scene(spec: mujoco.MjSpec) -> tuple[mujoco.MjModel, dict[str, dict[str, list[float]]]]:
    """Add a cup and two nominal fixed cameras to the licensed source model."""
    cup = spec.worldbody.add_geom()
    cup.name = "virtual_cup"
    cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    cup.size = [0.035, 0.055, 0.0]
    cup.pos = [0.0, 0.0, 0.055]
    cup.rgba = [0.1, 0.5, 0.9, 1.0]
    cameras = {
        "fixed_depth_01": {"position": [0.45, -0.45, 0.42], "target": [0.12, 0.0, 0.06]},
        "fixed_depth_02": {"position": [0.45, 0.45, 0.42], "target": [0.12, 0.0, 0.06]},
    }
    for name, definition in cameras.items():
        camera = spec.worldbody.add_camera()
        camera.name = name
        camera.pos = definition["position"]
        camera.quat = look_at_quat(np.array(definition["position"]), np.array(definition["target"])).tolist()
        camera.resolution = [320, 240]
        camera.fovy = 60.0
    return spec.compile(), cameras


def geometry_visible(model: mujoco.MjModel, data: mujoco.MjData, camera_name: str, cup_geom_id: int) -> bool:
    camera_id = model.camera(camera_name).id
    origin = data.cam_xpos[camera_id].copy()
    target = data.geom_xpos[cup_geom_id].copy()
    direction = target - origin
    direction /= np.linalg.norm(direction)
    geom_id = np.array([-1], dtype=np.int32)
    mujoco.mj_ray(model, data, origin, direction, None, True, -1, geom_id)
    return int(geom_id[0]) == cup_geom_id


def main() -> None:
    if OUTPUT_DIR.exists() or RESULT_PATH.exists() or RECORD_PATH.exists():
        raise FileExistsError("refusing to overwrite existing run evidence")
    if not SOURCE_SCENE.is_file():
        raise FileNotFoundError(SOURCE_SCENE)
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    OUTPUT_DIR.mkdir(parents=True)
    shutil.copy2(CONFIG_PATH, OUTPUT_DIR / "config.json")
    spec = mujoco.MjSpec.from_file(str(SOURCE_SCENE))
    source_model = spec.compile()
    estimated_reach = estimate_reach(source_model)
    workspace_radius = estimated_reach / 2.0
    model, virtual_cameras = add_virtual_scene(spec)
    data = mujoco.MjData(model)
    cup_geom_id = model.geom("virtual_cup").id
    derived_xml = spec.to_xml()
    (OUTPUT_DIR / "derived-scene.xml").write_text(derived_xml, encoding="utf-8")
    renderer = mujoco.Renderer(model, height=240, width=320)
    camera_names = ["wrist_cam", "fixed_depth_01", "fixed_depth_02"]
    episodes: list[dict[str, object]] = []
    try:
        for seed in range(10):
            generator = np.random.default_rng(seed)
            radius = workspace_radius * math.sqrt(float(generator.random()))
            angle = float(generator.uniform(-math.pi, math.pi))
            cup_position = np.array([radius * math.cos(angle), radius * math.sin(angle), 0.055])
            model.geom_pos[cup_geom_id] = cup_position
            mujoco.mj_forward(model, data)
            per_camera = {}
            for camera_name in camera_names:
                renderer.update_scene(data, camera=camera_name)
                rgb = renderer.render()
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=camera_name)
                depth = renderer.render()
                renderer.disable_depth_rendering()
                rgb_path = OUTPUT_DIR / f"seed-{seed:02d}-{camera_name}-rgb.png"
                depth_path = OUTPUT_DIR / f"seed-{seed:02d}-{camera_name}-depth.npy"
                Image.fromarray(rgb).save(rgb_path)
                np.save(depth_path, depth)
                per_camera[camera_name] = {
                    "rgb_path": rgb_path.relative_to(ROOT).as_posix(),
                    "depth_path": depth_path.relative_to(ROOT).as_posix(),
                    "finite_depth_fraction": float(np.isfinite(depth).mean()),
                    "geometry_visible": geometry_visible(model, data, camera_name, cup_geom_id),
                }
            episodes.append({"seed": seed, "cup_position": cup_position.tolist(), "cameras": per_camera})
    finally:
        renderer.close()
    metadata = {
        "run_id": RUN_ID,
        "model_source": {
            "repository_revision": "da76818e269b82289eba39808e2fb91d679d6994",
            "scene_sha256": sha256(SOURCE_DIR / "scene.xml"),
            "model_sha256": sha256(SOURCE_DIR / "so101.xml"),
            "license": "Apache-2.0"
        },
        "runtime": {"mujoco": mujoco.__version__, "python": platform.python_version()},
        "virtual_cameras": virtual_cameras,
        "reach_estimate": {"method": "10,000 seed-0 uniform joint-limit samples; radial gripperframe distance from baseframe", "estimated_maximum": estimated_reach, "workspace_radius": workspace_radius},
        "episodes": episodes
    }
    metadata_path = OUTPUT_DIR / "metadata.json"
    metadata_path.write_bytes(strict_json_bytes(metadata))
    duplicate_renderer = mujoco.Renderer(model, height=240, width=320)
    try:
        duplicate_renderer.update_scene(data, camera="wrist_cam")
        first_hash = hashlib.sha256(duplicate_renderer.render().tobytes()).hexdigest()
        duplicate_renderer.update_scene(data, camera="wrist_cam")
        second_hash = hashlib.sha256(duplicate_renderer.render().tobytes()).hexdigest()
    finally:
        duplicate_renderer.close()
    visible_counts = {name: sum(bool(episode["cameras"][name]["geometry_visible"]) for episode in episodes) for name in camera_names}
    result = {
        "schema_version": "first-robots/research-result/v1",
        "run_id": RUN_ID,
        "status": "observed_pending_human_review",
        "observations": {
            "episodes": len(episodes),
            "scene_loaded": True,
            "repeat_wrist_rgb_hash_equal": first_hash == second_hash,
            "geometry_visible_episode_count": visible_counts,
            "estimated_max_radial_reach": estimated_reach,
            "virtual_workspace_radius": workspace_radius
        },
        "limits": [
            "The half-reach workspace is based on an unvalidated candidate-model joint-limit sample, not a physical safety boundary.",
            "The wrist camera comes from the candidate model and both fixed-depth cameras are virtual nominal viewpoints.",
            "No grasping, IK, collision evaluation, physical calibration, perception metric, controller, sim-to-real result, or multi-robot capability was evaluated."
        ],
        "review": {"status": "pending_human_review", "adopted": False}
    }
    record = {
        "schema_version": "first-robots/experiment-record/v1",
        "run_id": RUN_ID,
        "kind": "formal_virtual_observation_preflight",
        "result": {"path": RESULT_PATH.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
        "review": {"status": "pending_human_review"}
    }
    write_json_pair(first_path=RECORD_PATH, first_content=record, second_path=RESULT_PATH, second_content=result)
    with RunRecorder(
        run_id=RUN_ID,
        objective="Verify repeatable virtual three-view observation artifacts for an authorized candidate SO-101 MuJoCo model without asserting hardware equivalence.",
        script=Path(__file__),
        repo_root=ROOT,
        output_root=ROOT / "research" / "runs",
        parameters={"seeds": list(range(10)), "reach_samples": 10_000, "workspace_radius_factor": 0.5, "actuation": "disabled"},
        random_seed=0,
        inputs=[
            {"kind": "configuration", "path": CONFIG_PATH.relative_to(ROOT).as_posix(), "sha256": sha256(CONFIG_PATH)},
            {"kind": "candidate_model", "path": SOURCE_SCENE.relative_to(ROOT).as_posix(), "sha256": sha256(SOURCE_SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994", "license": "Apache-2.0"}
        ],
        baseline={"kind": "comparison", "reference": "Render the final identical wrist-camera state twice; equality only tests this run's renderer repeatability."},
        level="formal",
    ) as recorder:
        recorder.add_metric("episode_count", len(episodes), "episodes")
        recorder.add_metric("estimated_max_radial_reach", estimated_reach, "model_units")
        recorder.add_metric("virtual_workspace_radius", workspace_radius, "model_units")
        recorder.add_metric("repeat_wrist_rgb_hash_equal", first_hash == second_hash, "boolean")
        for camera_name, count in visible_counts.items():
            recorder.add_metric(f"{camera_name}_geometry_visible_episodes", count, "episodes")
        for path in sorted(OUTPUT_DIR.iterdir()):
            if path.is_file():
                recorder.add_artifact(path)
        recorder.add_artifact(RESULT_PATH)
        recorder.add_artifact(RECORD_PATH)
        recorder.complete(
            passed=True,
            criteria="All 10 seeds emitted reviewable RGB/depth artifacts; model loading and repeated wrist rendering completed. This is not a hardware or capability acceptance criterion.",
            review_status="pending_human_review",
            note="Virtual-adapter evidence only; no conclusion is adopted."
        )


if __name__ == "__main__":
    main()
