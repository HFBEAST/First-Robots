"""Research-only candidate-camera projection diagnosis; never controls hardware."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260916-002-virtual-wrist-projection-diagnosis"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260916-002-virtual-wrist-projection-diagnosis.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
SEGMENT = ROOT / "research/artifacts/EXP-20260902-002-virtual-wrist-segment-contact-scan/segment-contact-scan.json"
RASTER = ROOT / "research/artifacts/EXP-20260902-003-virtual-wrist-raster-visibility/raster-visibility.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    cfg = json.loads(CFG.read_text())
    segment = json.loads(SEGMENT.read_text())
    raster = json.loads(RASTER.read_text())
    raster_rows = {(row["configuration_index"], row["cup_index"]): row for row in raster["rows"]}
    expected_pairs = len(segment["rows"]) * len(segment["cups"])
    if len(raster_rows) != expected_pairs:
        raise ValueError("source raster table does not contain one unique row per configuration-cup pair")
    rec = RunRecorder(
        run_id=RID,
        objective="Record candidate-camera center projection conditions with an explicit target-placement invariant and without asserting visibility capability.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_segment_table", "path": SEGMENT.relative_to(ROOT).as_posix(), "sha256": file_hash(SEGMENT)},
            {"kind": "source_raster_table", "path": RASTER.relative_to(ROOT).as_posix(), "sha256": file_hash(RASTER)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994"},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260916-001 is retained as invalid because its cup placement invariant was absent."},
        level="formal",
    )
    with rec:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(cfg))
        spec = mujoco.MjSpec.from_file(str(SCENE))
        cup = spec.worldbody.add_geom()
        cup.name = "virtual_cup"
        cup.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        cup.size = [.035, .055, 0]
        cup.pos = [0, 0, .055]
        model = spec.compile()
        data = mujoco.MjData(model)
        cup_id = model.geom("virtual_cup").id
        camera_id = model.camera(cfg["protocol"]["camera"]).id
        width = cfg["protocol"]["source_raster_render_size"]["width"]
        height = cfg["protocol"]["source_raster_render_size"]["height"]
        aspect = width / height
        fovy_degrees = float(model.cam_fovy[camera_id])
        tan_half_vertical = math.tan(math.radians(fovy_degrees) / 2.0)
        tan_half_horizontal = tan_half_vertical * aspect
        near = float(model.vis.map.znear * model.stat.extent)
        far = float(model.vis.map.zfar * model.stat.extent)
        rows = []
        for source_row in segment["rows"]:
            for cup_index, position in enumerate(segment["cups"]):
                expected_position = np.asarray(position, dtype=float)
                data.qpos[:] = source_row["qpos"]
                model.geom_pos[cup_id] = expected_position
                mujoco.mj_forward(model, data)
                world_center = data.geom_xpos[cup_id].copy()
                if not np.allclose(world_center, expected_position, rtol=0.0, atol=1e-12):
                    raise RuntimeError(f"virtual_cup position invariant failed for configuration={source_row['sample_index']} cup={cup_index}")
                camera_position = data.cam_xpos[camera_id].copy()
                world_to_camera = data.cam_xmat[camera_id].reshape(3, 3).T
                camera_coordinates = world_to_camera @ (world_center - camera_position)
                forward_depth = float(-camera_coordinates[2])
                behind = forward_depth <= 0.0
                depth_clipped = (not behind) and (forward_depth < near or forward_depth > far)
                outside_frustum = (not behind) and (
                    abs(float(camera_coordinates[0])) > forward_depth * tan_half_horizontal
                    or abs(float(camera_coordinates[1])) > forward_depth * tan_half_vertical
                )
                inside_frustum_and_clip = not behind and not depth_clipped and not outside_frustum
                prior = raster_rows[(source_row["sample_index"], cup_index)]
                rows.append({
                    "configuration_index": source_row["sample_index"], "cup_index": cup_index,
                    "input_position": expected_position.tolist(), "world_center": world_center.tolist(),
                    "camera_position": camera_position.tolist(), "camera_coordinates": camera_coordinates.tolist(),
                    "forward_depth": forward_depth, "behind_camera": behind,
                    "outside_candidate_depth_clip": depth_clipped,
                    "outside_candidate_perspective_frustum": outside_frustum,
                    "inside_candidate_frustum_and_clip": inside_frustum_and_clip,
                    "ray_visible": prior["ray_visible"], "raster_visible": prior["raster_visible"],
                    "target_pixels": prior["target_pixels"],
                })
        counts = {
            "behind_camera": sum(row["behind_camera"] for row in rows),
            "outside_candidate_depth_clip": sum(row["outside_candidate_depth_clip"] for row in rows),
            "outside_candidate_perspective_frustum": sum(row["outside_candidate_perspective_frustum"] for row in rows),
            "inside_candidate_frustum_and_clip": sum(row["inside_candidate_frustum_and_clip"] for row in rows),
            "inside_frustum_raster_invisible": sum(row["inside_candidate_frustum_and_clip"] and not row["raster_visible"] for row in rows),
            "ray_visible_inside_frustum_raster_invisible": sum(row["ray_visible"] and row["inside_candidate_frustum_and_clip"] and not row["raster_visible"] for row in rows),
        }
        meta = {
            "run_id": RID, "mujoco": mujoco.__version__, "source_runs": [segment["run_id"], raster["run_id"]],
            "camera": {"name": cfg["protocol"]["camera"], "fovy_degrees": fovy_degrees, "aspect": aspect,
                       "candidate_near_distance": near, "candidate_far_distance": far},
            "placement_invariant": "passed for every row", "rows": rows, "counts": counts,
        }
        (OUT / "projection-diagnosis.json").write_bytes(strict_json_bytes(meta))
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {"comparisons": len(rows), "placement_invariant_samples": len(rows), **counts},
            "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_wrist_camera_projection_diagnosis",
            "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
            "review": {"status": "pending_human_review"},
        }
        write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)
        for path in sorted(OUT.iterdir()):
            rec.add_artifact(path)
        rec.add_artifact(RESULT)
        rec.add_artifact(RECORD)
        rec.add_metric("comparisons", len(rows), "configuration-cup pairs")
        rec.add_metric("placement_invariant_samples", len(rows), "configuration-cup pairs")
        for name, value in counts.items():
            rec.add_metric(name, value, "configuration-cup pairs")
        rec.complete(
            passed=True,
            criteria="All source configuration-cup pairs satisfied target placement and were diagnosed; no visibility or capability criterion.",
            review_status="pending_human_review", note="Candidate-model camera-center projection diagnosis only.",
        )


if __name__ == "__main__":
    main()
