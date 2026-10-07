"""Create one labeled XY review map from a frozen virtual coordinate state."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair


RID = "EXP-20260921-013-world-coordinate-xy-review-map"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs" / f"{RID}.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists():
        raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config = json.loads(CFG.read_text(encoding="utf-8"))
    predecessor = config["blocking_predecessor"]
    result_path = ROOT / predecessor["result_path"]
    state_path = ROOT / predecessor["coordinate_state_path"]
    if sha256_file(result_path) != predecessor["result_sha256"] or sha256_file(state_path) != predecessor["coordinate_state_sha256"] or sha256_file(SCENE) != config["model"]["scene_sha256"]:
        raise RuntimeError("frozen input hash mismatch")
    state = json.loads(state_path.read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    gripper_position = data.site_xpos[model.site("gripperframe").id].copy().tolist()
    contract = json.loads((ROOT / "research/configs/EXP-20260921-012-world-coordinate-contract-review.json").read_text(encoding="utf-8"))["world_coordinate_contract"]
    map_config = config["map"]
    x_min, x_max = map_config["world_xy_bounds_m"]["x"]
    y_min, y_max = map_config["world_xy_bounds_m"]["y"]
    width, height = map_config["image_size_px"]
    margin = 70
    scale_x = (width - 2 * margin) / (x_max - x_min)
    scale_y = (height - 2 * margin) / (y_max - y_min)
    def point(x: float, y: float) -> tuple[int, int]:
        return (round(margin + (x - x_min) * scale_x), round(height - margin - (y - y_min) * scale_y))
    recorder = RunRecorder(run_id=RID, objective="Create a labeled XY coordinate review map from the frozen virtual state.", script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs", parameters=map_config, random_seed=None, inputs=[{"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": sha256_file(CFG)}, {"kind": "predecessor_result", "path": result_path.relative_to(ROOT).as_posix(), "sha256": sha256_file(result_path)}, {"kind": "coordinate_state", "path": state_path.relative_to(ROOT).as_posix(), "sha256": sha256_file(state_path)}, {"kind": "candidate_model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": sha256_file(SCENE)}], baseline={"kind": "frozen_coordinate_state", "reference": predecessor["run_id"]}, level="formal")
    with recorder:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(config))
        image = Image.new("RGB", (width, height), (248, 248, 248))
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()
        for value in np.arange(np.ceil(x_min / map_config["grid_step_m"]) * map_config["grid_step_m"], x_max + 1e-12, map_config["grid_step_m"]):
            px = point(float(value), 0)[0]
            draw.line((px, margin, px, height - margin), fill=(220, 220, 220))
        for value in np.arange(np.ceil(y_min / map_config["grid_step_m"]) * map_config["grid_step_m"], y_max + 1e-12, map_config["grid_step_m"]):
            py = point(0, float(value))[1]
            draw.line((margin, py, width - margin, py), fill=(220, 220, 220))
        draw.text((margin, 20), map_config["title"], fill=(0, 0, 0), font=font)
        table = contract["table"]
        tx0, ty0 = point(-table["half_extents_m"][0], -table["half_extents_m"][1])
        tx1, ty1 = point(table["half_extents_m"][0], table["half_extents_m"][1])
        draw.rectangle((tx0, ty1, tx1, ty0), fill=(226, 211, 181), outline=(100, 85, 55), width=2)
        draw.text((tx0 + 8, ty1 + 8), "table: center (0,0,-0.025), top Z=0", fill=(80, 65, 35), font=font)
        barrier = contract["barrier"]
        bx0, by0 = point(barrier["center_m"][0] - barrier["half_extents_m"][0], barrier["center_m"][1] - barrier["half_extents_m"][1])
        bx1, by1 = point(barrier["center_m"][0] + barrier["half_extents_m"][0], barrier["center_m"][1] + barrier["half_extents_m"][1])
        draw.rectangle((bx0, by1, bx1, by0), fill=(0, 160, 190), outline=(0, 80, 100), width=2)
        draw.text((bx1 + 5, by1 + 5), "barrier (0.28,0,0.16)", fill=(0, 70, 90), font=font)
        cup = contract["cup"]
        cx, cy = point(cup["center_m"][0], cup["center_m"][1])
        radius = round(cup["radius_m"] * min(scale_x, scale_y))
        draw.ellipse((cx - radius, cy - radius, cx + radius, cy + radius), fill=(215, 40, 160), outline=(110, 0, 75), width=2)
        draw.text((cx + 8, cy + 8), "cup (0.12,0,0.055)", fill=(110, 0, 75), font=font)
        base = state["world_positions_m"]["baseframe"]
        ox, oy = point(base[0], base[1])
        draw.ellipse((ox - 6, oy - 6, ox + 6, oy + 6), fill=(0, 0, 0))
        draw.text((ox + 8, oy + 8), "baseframe (0,0,0)", fill=(0, 0, 0), font=font)
        gx, gy = point(gripper_position[0], gripper_position[1])
        draw.rectangle((gx - 5, gy - 5, gx + 5, gy + 5), fill=(255, 190, 0), outline=(120, 80, 0))
        draw.text((gx + 8, gy - 16), f"default gripperframe ({gripper_position[0]:.3f},{gripper_position[1]:.3f},{gripper_position[2]:.3f})", fill=(120, 80, 0), font=font)
        for name, position in state["compiled_camera_positions_m"].items():
            px, py = point(position[0], position[1])
            color = (210, 70, 30) if name.startswith("fixed") else (90, 50, 170)
            draw.regular_polygon((px, py, 7), n_sides=3, rotation=0, fill=color)
            draw.text((px + 8, py + 4), f"{name} ({position[0]:.2f},{position[1]:.2f},{position[2]:.2f})", fill=color, font=font)
        draw.line((ox, oy, point(0.18, 0)[0], oy), fill=(220, 0, 0), width=3)
        draw.text((point(0.18, 0)[0] + 4, oy + 4), "+X", fill=(220, 0, 0), font=font)
        draw.line((ox, oy, ox, point(0, 0.18)[1]), fill=(0, 150, 0), width=3)
        draw.text((ox + 4, point(0, 0.18)[1] - 14), "+Y", fill=(0, 150, 0), font=font)
        draw.text((margin, height - 42), "All positions in metres. XY projection; Z is embedded in labels.", fill=(0, 0, 0), font=font)
        output = OUT / "world-coordinate-xy-map.png"
        image.save(output)
        labels = ["baseframe", "default gripperframe", "table", "cup", "barrier", "fixed_depth_01", "fixed_depth_02", "review_isometric", "review_top", "+X", "+Y"]
        assessment = {"map_path": output.relative_to(ROOT).as_posix(), "labels": labels, "default_gripperframe_world_position_m": gripper_position, "coordinate_state_source": predecessor["coordinate_state_path"], "side_grasp_target_pose": "not_defined"}
        (OUT / "map-assessment.json").write_bytes(strict_json_bytes(assessment))
        valid = bool(np.any(np.asarray(image) != np.asarray(image)[0, 0]))
        result = {"schema_version": "first-robots/research-result/v1", "run_id": RID, "status": "passed_pending_human_review" if valid else "blocked_pending_human_review", "observations": assessment, "limits": config["non_claims"], "review": {"status": "pending_human_review", "adopted": False}}
        execution_record = {"schema_version": "first-robots/experiment-record/v1", "run_id": RID, "kind": "formal_world_coordinate_xy_review_map", "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))}, "review": {"status": "pending_human_review", "adopted": False}}
        write_json_pair(first_path=EXECUTION_RECORD, first_content=execution_record, second_path=RESULT, second_content=result)
        for artifact in sorted(OUT.iterdir()):
            recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD)
        recorder.add_artifact(RESULT)
        recorder.add_metric("coordinate_map_nonuniform", valid, "boolean")
        recorder.complete(passed=valid, criteria="Frozen coordinate state produced a labeled nonuniform XY review-map artifact.", review_status="pending_human_review", note="Coordinate review visualization only; no reachability or grasp conclusion.")


if __name__ == "__main__":
    main()
