"""Research-only fixed relative-pose golden case; never controls hardware."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260920-001-virtual-wrist-relative-pose-golden-case"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def single_attempt(cfg: dict, output: Path, mask_output: Path) -> None:
    if sha256_file(SCENE) != cfg["model"]["scene_sha256"]:
        raise RuntimeError("source scene SHA-256 does not match the frozen configuration")
    protocol = cfg["protocol"]
    spec = mujoco.MjSpec.from_file(str(SCENE))
    cup = spec.worldbody.add_geom()
    cup.name = "virtual_cup"
    cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    cup.size = [protocol["cup"]["radius_m"], protocol["cup"]["half_height_m"], 0]
    cup.pos = [0, 0, protocol["cup"]["half_height_m"]]
    cup.rgba = [0.1, 0.5, 0.9, 1]
    model = spec.compile()
    data = mujoco.MjData(model)
    cup_id = model.geom("virtual_cup").id
    camera_id = model.camera(protocol["camera"]).id
    data.qpos[:] = protocol["qpos"]
    mujoco.mj_forward(model, data)
    camera_position = data.cam_xpos[camera_id].copy()
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3).copy()
    requested_camera_relative = np.asarray(protocol["cup"]["center_in_camera_coordinates_m"], dtype=float)
    requested_world_position = camera_position + camera_rotation @ requested_camera_relative
    model.geom_pos[cup_id] = requested_world_position
    mujoco.mj_forward(model, data)
    observed_world_position = data.geom_xpos[cup_id].copy()
    observed_camera_relative = camera_rotation.T @ (observed_world_position - camera_position)
    world_position_invariant = bool(np.allclose(observed_world_position, requested_world_position, rtol=0.0, atol=1e-12))
    relative_position_invariant = bool(np.allclose(observed_camera_relative, requested_camera_relative, rtol=0.0, atol=1e-12))
    if not world_position_invariant or not relative_position_invariant:
        raise RuntimeError("virtual_cup world or camera-relative position invariant failed")
    renderer = mujoco.Renderer(model, height=protocol["renderer"]["height"], width=protocol["renderer"]["width"])
    try:
        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=protocol["camera"])
        segmentation = renderer.render()
    finally:
        renderer.close()
    target_mask = (segmentation[:, :, 0] == cup_id) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
    np.save(mask_output, target_mask)
    observation = {
        "run_id": RID,
        "runtime": {"python": sys.version, "platform": platform.platform(), "mujoco": mujoco.__version__},
        "source_scene_sha256": sha256_file(SCENE),
        "input": {
            "qpos": protocol["qpos"],
            "requested_camera_relative_center_m": requested_camera_relative.tolist(),
            "requested_world_center_m": requested_world_position.tolist()
        },
        "invariants": {
            "world_position": world_position_invariant,
            "camera_relative_position": relative_position_invariant,
            "observed_world_center_m": observed_world_position.tolist(),
            "observed_camera_relative_center_m": observed_camera_relative.tolist()
        },
        "render": {
            "target_pixels": int(np.count_nonzero(target_mask)),
            "segmentation_sha256": "sha256:" + hashlib.sha256(segmentation.tobytes()).hexdigest(),
            "mask_path": mask_output.relative_to(ROOT).as_posix()
        }
    }
    output.write_bytes(strict_json_bytes(observation))


def write_result(cfg: dict, observations: dict, passed: bool, failure: str | None = None) -> None:
    result = {
        "schema_version": "first-robots/research-result/v1",
        "run_id": RID,
        "status": "passed_pending_human_review" if passed else "blocked_pending_human_review",
        "observations": observations,
        "limits": cfg["non_claims"],
        "review": {"status": "pending_human_review", "adopted": False}
    }
    if failure is not None:
        result["failure"] = failure
    record = {
        "schema_version": "first-robots/experiment-record/v1",
        "run_id": RID,
        "kind": "formal_virtual_wrist_relative_pose_golden_case",
        "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
        "review": {"status": "pending_human_review"}
    }
    write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--single-attempt", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--mask-output", type=Path)
    args = parser.parse_args()
    cfg = json.loads(CFG.read_text())
    if args.single_attempt:
        if args.output is None or args.mask_output is None:
            parser.error("--single-attempt requires --output and --mask-output")
        single_attempt(cfg, args.output, args.mask_output)
        return
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    rec = RunRecorder(
        run_id=RID,
        objective="Establish one fixed candidate-model wrist-camera relative-pose golden case before any position sweep.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=None,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE), "revision": cfg["model"]["repository_revision"]}
        ],
        baseline={"kind": "golden_case", "reference": "No earlier raster observation is a baseline for this fixed camera-relative pose."},
        level="formal"
    )
    with rec:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(cfg))
        attempts = []
        for index in range(cfg["protocol"]["fresh_process_attempts"]):
            output = OUT / f"attempt-{index:02d}.json"
            mask_output = OUT / f"attempt-{index:02d}-target-mask.npy"
            completed = subprocess.run(
                [sys.executable, str(Path(__file__)), "--single-attempt", "--output", str(output), "--mask-output", str(mask_output)],
                text=True, capture_output=True, check=False
            )
            if completed.returncode != 0:
                failure = {
                    "attempt_index": index,
                    "returncode": completed.returncode,
                    "stdout": completed.stdout,
                    "stderr": completed.stderr
                }
                (OUT / f"attempt-{index:02d}-failure.json").write_bytes(strict_json_bytes(failure))
                observations = {"completed_attempts": index, "required_attempts": cfg["protocol"]["fresh_process_attempts"]}
                write_result(cfg, observations, passed=False, failure="a fresh-process attempt failed; see attempt failure artifact")
                for path in sorted(OUT.iterdir()):
                    rec.add_artifact(path)
                rec.add_artifact(RESULT)
                rec.add_artifact(RECORD)
                rec.complete(passed=False, criteria="A child-process failure blocks the golden-case gate.", review_status="pending_human_review", note="No downstream relative-position experiment is authorized.")
                return
            attempts.append(json.loads(output.read_text()))
        pixel_counts = [attempt["render"]["target_pixels"] for attempt in attempts]
        segmentation_hashes = [attempt["render"]["segmentation_sha256"] for attempt in attempts]
        passed = (
            all(attempt["invariants"]["world_position"] and attempt["invariants"]["camera_relative_position"] for attempt in attempts)
            and all(count > 0 for count in pixel_counts)
            and len(set(pixel_counts)) == 1
            and len(set(segmentation_hashes)) == 1
        )
        observations = {
            "fresh_process_attempts": len(attempts),
            "world_position_invariants_passed": sum(attempt["invariants"]["world_position"] for attempt in attempts),
            "camera_relative_position_invariants_passed": sum(attempt["invariants"]["camera_relative_position"] for attempt in attempts),
            "target_pixels_by_attempt": pixel_counts,
            "segmentation_sha256_by_attempt": segmentation_hashes,
            "all_target_pixel_counts_equal": len(set(pixel_counts)) == 1,
            "all_segmentation_sha256_equal": len(set(segmentation_hashes)) == 1
        }
        write_result(cfg, observations, passed=passed)
        for path in sorted(OUT.iterdir()):
            rec.add_artifact(path)
        rec.add_artifact(RESULT)
        rec.add_artifact(RECORD)
        rec.add_metric("fresh_process_attempts", len(attempts), "processes")
        rec.add_metric("world_position_invariants_passed", observations["world_position_invariants_passed"], "processes")
        rec.add_metric("camera_relative_position_invariants_passed", observations["camera_relative_position_invariants_passed"], "processes")
        rec.add_metric("target_pixels", pixel_counts[0], "pixels")
        rec.complete(
            passed=passed,
            criteria="All three independent attempts must satisfy position invariants and have identical nonzero segmentation output.",
            review_status="pending_human_review",
            note="A failed gate blocks downstream relative-position sweeps."
        )


if __name__ == "__main__":
    main()
