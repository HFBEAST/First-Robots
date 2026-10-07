"""Research-only one-step wrist-camera relative-Y test; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260920-004-virtual-wrist-relative-y-step"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    cfg = json.loads(CFG.read_text())
    predecessor_path = ROOT / cfg["blocking_predecessor"]["result_path"]
    predecessor = json.loads(predecessor_path.read_text())
    if file_hash(predecessor_path) != cfg["blocking_predecessor"]["result_sha256"] or predecessor["status"] != "passed_pending_human_review":
        raise RuntimeError("X-step predecessor is not the frozen technically passed result")
    if file_hash(SCENE) != cfg["model"]["scene_sha256"]:
        raise RuntimeError("source scene SHA-256 does not match frozen configuration")
    protocol = cfg["protocol"]
    rec = RunRecorder(run_id=RID, objective="Test one wrist-camera lateral Y step after the frozen relative-X step.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=protocol, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)}, {"kind": "predecessor_result", "path": predecessor_path.relative_to(ROOT).as_posix(), "sha256": file_hash(predecessor_path)}, {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": cfg["model"]["repository_revision"]}], baseline={"kind": "prior_relative_x_step", "reference": cfg["blocking_predecessor"]["run_id"]}, level="formal")
    with rec:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(cfg))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        cup = spec.worldbody.add_geom(); cup.name = "virtual_cup"; cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.size = [protocol["cup"]["radius_m"], protocol["cup"]["half_height_m"], 0]; cup.pos = [0, 0, protocol["cup"]["half_height_m"]]
        model = spec.compile(); data = mujoco.MjData(model); cup_id = model.geom("virtual_cup").id; camera_id = model.camera(protocol["camera"]).id
        data.qpos[:] = protocol["qpos"]; mujoco.mj_forward(model, data)
        camera_position = data.cam_xpos[camera_id].copy(); camera_rotation = data.cam_xmat[camera_id].reshape(3, 3).copy()
        tested_relative = np.asarray(protocol["independent_variable"]["tested_m"], dtype=float); tested_world = camera_position + camera_rotation @ tested_relative
        model.geom_pos[cup_id] = tested_world; mujoco.mj_forward(model, data)
        observed_world = data.geom_xpos[cup_id].copy(); observed_relative = camera_rotation.T @ (observed_world - camera_position)
        world_ok = bool(np.allclose(observed_world, tested_world, rtol=0.0, atol=1e-12)); relative_ok = bool(np.allclose(observed_relative, tested_relative, rtol=0.0, atol=1e-12))
        if not world_ok or not relative_ok: raise RuntimeError("tested relative-position invariant failed")
        renderer = mujoco.Renderer(model, height=protocol["renderer"]["height"], width=protocol["renderer"]["width"])
        try:
            renderer.enable_segmentation_rendering(); renderer.update_scene(data, camera=protocol["camera"]); segmentation = renderer.render()
        finally:
            renderer.close()
        mask = (segmentation[:, :, 0] == cup_id) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)); np.save(OUT / "target-mask.npy", mask)
        target_pixels = int(np.count_nonzero(mask)); baseline_pixels = predecessor["observations"]["tested_target_pixels"]
        observation = {"baseline_target_pixels": baseline_pixels, "tested_target_pixels": target_pixels, "pixel_difference_from_baseline": target_pixels - baseline_pixels, "input_tested_camera_relative_center_m": tested_relative.tolist(), "observed_tested_camera_relative_center_m": observed_relative.tolist(), "world_position_invariant": world_ok, "camera_relative_position_invariant": relative_ok, "segmentation_sha256": "sha256:" + hashlib.sha256(segmentation.tobytes()).hexdigest()}
        (OUT / "relative-y-step.json").write_bytes(strict_json_bytes(observation))
        passed = target_pixels > 0
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if passed else "blocked_pending_human_review", "observations": observation, "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_virtual_wrist_relative_y_step", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review"}}
        write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)
        for path in sorted(OUT.iterdir()): rec.add_artifact(path)
        rec.add_artifact(RESULT); rec.add_artifact(RECORD); rec.add_metric("tested_target_pixels", target_pixels, "pixels"); rec.add_metric("pixel_difference_from_baseline", target_pixels - baseline_pixels, "pixels")
        rec.complete(passed=passed, criteria="The frozen predecessor and source must match, both position invariants must pass, and the tested condition must have nonzero target pixels.", review_status="pending_human_review", note="Pixel difference is recorded without a field-of-view or causal interpretation.")


if __name__ == "__main__": main()
