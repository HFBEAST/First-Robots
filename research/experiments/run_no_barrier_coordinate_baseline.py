"""Freeze one no-barrier virtual coordinate baseline; never moves a robot or uses hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260921-015-no-barrier-coordinate-baseline"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"

def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()

def look_at_quaternion(position: np.ndarray, target: np.ndarray) -> list[float]:
    z_axis = -(target - position); z_axis /= np.linalg.norm(z_axis)
    x_axis = np.cross(np.array([0.0, 0.0, 1.0]), z_axis); x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    output = np.empty(4); mujoco.mju_mat2Quat(output, np.column_stack((x_axis, y_axis, z_axis)).reshape(9)); return output.tolist()

def add_camera(spec: mujoco.MjSpec, name: str, camera: dict, renderer: dict) -> None:
    view = spec.worldbody.add_camera(); view.name = name; view.pos = camera["position_m"]
    view.quat = look_at_quaternion(np.asarray(camera["position_m"]), np.asarray(camera["target_m"])); view.resolution = [renderer["width"], renderer["height"]]; view.fovy = 60

def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists(): raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8")); prior = ROOT / config["blocking_predecessor"]["result_path"]
    if sha256_file(prior) != config["blocking_predecessor"]["result_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]: raise RuntimeError("frozen input hash mismatch")
    scenario = config["scenario"]
    recorder = RunRecorder(run_id=RID, objective="Freeze no-barrier world coordinates before multi-range coordinate and Agent protocol simulation.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=scenario, random_seed=None, inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":prior.relative_to(ROOT).as_posix(),"sha256":sha256_file(prior)},{"kind":"candidate_model","path":SCENE.relative_to(ROOT).as_posix(),"sha256":sha256_file(SCENE)}], baseline={"kind":"no_barrier_coordinate_baseline","reference":config["blocking_predecessor"]["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True); (OUT / "config.json").write_bytes(strict_json_bytes(config)); spec = mujoco.MjSpec.from_file(str(SCENE))
        table = spec.worldbody.add_geom(); table.name = scenario["table"]["name"]; table.type = mujoco.mjtGeom.mjGEOM_BOX; table.size = scenario["table"]["half_extents_m"]; table.pos = scenario["table"]["center_m"]; table.rgba = scenario["table"]["rgba"]; table.contype = 0; table.conaffinity = 0
        cup = spec.worldbody.add_geom(); cup.name = scenario["cup_a"]["name"]; cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER; cup.size = [scenario["cup_a"]["radius_m"],scenario["cup_a"]["half_height_m"],0]; cup.pos = scenario["cup_a"]["center_m"]; cup.rgba = scenario["cup_a"]["rgba"]; cup.contype = 0; cup.conaffinity = 0
        for name, camera in scenario["fixed_cameras"].items(): add_camera(spec,name,camera,scenario["renderer"])
        for name, camera in scenario["review_cameras"].items(): add_camera(spec,name,camera,scenario["renderer"])
        model = spec.compile(); data = mujoco.MjData(model); data.qpos[:] = scenario["robot_qpos_rad"]; mujoco.mj_forward(model,data)
        barrier_absent = all(model.geom(i).name != "vlab_barrier" for i in range(model.ngeom))
        world_positions = {"baseframe":data.site_xpos[model.site("baseframe").id].tolist(),scenario["table"]["name"]:data.geom_xpos[model.geom(scenario["table"]["name"]).id].tolist(),scenario["cup_a"]["name"]:data.geom_xpos[model.geom(scenario["cup_a"]["name"]).id].tolist()}
        expected = {"baseframe":scenario["baseframe_world_position_m"],scenario["table"]["name"]:scenario["table"]["center_m"],scenario["cup_a"]["name"]:scenario["cup_a"]["center_m"]}
        positions_match = all(np.allclose(world_positions[name],value,rtol=0,atol=1e-12) for name,value in expected.items()); table_top_matches = scenario["table"]["center_m"][2] + scenario["table"]["half_extents_m"][2] == scenario["table"]["top_z_m"] == 0
        camera_positions = {name:model.cam_pos[model.camera(name).id].tolist() for name in (*scenario["fixed_cameras"],*scenario["review_cameras"])}
        cameras_match = all(np.allclose(camera_positions[name],camera["position_m"],rtol=0,atol=1e-12) for group in (scenario["fixed_cameras"],scenario["review_cameras"]) for name,camera in group.items())
        images = {}; renderer = mujoco.Renderer(model,height=scenario["renderer"]["height"],width=scenario["renderer"]["width"])
        try:
            for name in scenario["review_cameras"]:
                renderer.update_scene(data,camera=name); image = renderer.render(); Image.fromarray(image).save(OUT/f"{name}-rgb.png"); images[name] = {"path":(OUT/f"{name}-rgb.png").relative_to(ROOT).as_posix(),"nonuniform_pixels":bool(np.any(image != image[0,0]))}
        finally: renderer.close()
        passed = barrier_absent and positions_match and table_top_matches and cameras_match and all(item["nonuniform_pixels"] for item in images.values())
        assessment = {"barrier_absent":barrier_absent,"world_positions_m":world_positions,"compiled_camera_positions_m":camera_positions,"positions_match_configuration":positions_match,"table_top_matches_world_z_zero":table_top_matches,"camera_positions_match_configuration":cameras_match,"review_images":images,"cup_transfer_assessment":"not_evaluated","grasp_reachability_assessment":"not_evaluated"}
        (OUT/"coordinate-state.json").write_bytes(strict_json_bytes(assessment)); result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":assessment,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}}; execution_record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_no_barrier_coordinate_baseline","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}}
        write_json_pair(first_path=EXECUTION_RECORD,first_content=execution_record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()): recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD); recorder.add_artifact(RESULT); recorder.add_metric("no_barrier_coordinate_baseline_valid",passed,"boolean"); recorder.complete(passed=passed,criteria="No barrier compiles; frozen coordinates/cameras match; review images complete.",review_status="pending_human_review",note="No-barrier coordinate baseline only; no transfer or robot action.")

if __name__ == "__main__": main()
