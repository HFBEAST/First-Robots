"""Research-only comparison of virtual ray and raster visibility; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260902-003-virtual-wrist-raster-visibility"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260902-003-virtual-wrist-raster-visibility.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
SOURCE = ROOT / "research/artifacts/EXP-20260902-002-virtual-wrist-segment-contact-scan/segment-contact-scan.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def ray_visible(model: mujoco.MjModel, data: mujoco.MjData, cup_id: int) -> bool:
    origin = data.cam_xpos[model.camera("wrist_cam").id].copy()
    direction = data.geom_xpos[cup_id] - origin
    direction /= np.linalg.norm(direction)
    hit = np.array([-1], dtype=np.int32)
    mujoco.mj_ray(model, data, origin, direction, None, True, -1, hit)
    return int(hit[0]) == cup_id


def main() -> None:
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    cfg = json.loads(CFG.read_text())
    source = json.loads(SOURCE.read_text())
    rec = RunRecorder(
        run_id=RID,
        objective="Compare candidate-model ray hits with virtual wrist-camera segmentation pixels without asserting perception or hardware capability.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_segment_table", "path": SOURCE.relative_to(ROOT).as_posix(), "sha256": file_hash(SOURCE)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994"},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260902-002 reports ray-derived geometric visibility only."},
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
        cup.rgba = [.1, .5, .9, 1]
        model = spec.compile()
        data = mujoco.MjData(model)
        cup_id = model.geom("virtual_cup").id
        renderer = mujoco.Renderer(model, height=cfg["protocol"]["renderer"]["height"], width=cfg["protocol"]["renderer"]["width"])
        renderer.enable_segmentation_rendering()
        rows = []
        try:
            for source_row in source["rows"]:
                for cup_index, position in enumerate(source["cups"]):
                    data.qpos[:] = source_row["qpos"]
                    model.geom_pos[cup_id] = position
                    mujoco.mj_forward(model, data)
                    ray = ray_visible(model, data, cup_id)
                    renderer.update_scene(data, camera=cfg["protocol"]["renderer"]["camera"])
                    segmentation = renderer.render()
                    target_pixels = int(np.count_nonzero(
                        (segmentation[:, :, 0] == cup_id)
                        & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
                    ))
                    rows.append({
                        "configuration_index": source_row["sample_index"], "cup_index": cup_index,
                        "ray_visible": ray, "target_pixels": target_pixels,
                        "raster_visible": target_pixels > 0,
                    })
        finally:
            renderer.close()
        counts = {
            "ray_and_raster_visible": sum(row["ray_visible"] and row["raster_visible"] for row in rows),
            "ray_visible_raster_invisible": sum(row["ray_visible"] and not row["raster_visible"] for row in rows),
            "ray_invisible_raster_visible": sum(not row["ray_visible"] and row["raster_visible"] for row in rows),
            "ray_and_raster_invisible": sum(not row["ray_visible"] and not row["raster_visible"] for row in rows),
        }
        meta = {"run_id": RID, "mujoco": mujoco.__version__, "source_run": source["run_id"], "rows": rows, "counts": counts}
        (OUT / "raster-visibility.json").write_bytes(strict_json_bytes(meta))
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {"comparisons": len(rows), **counts, "maximum_target_pixels": max(row["target_pixels"] for row in rows)},
            "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_ray_raster_visibility_comparison",
            "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
            "review": {"status": "pending_human_review"},
        }
        write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)
        for path in sorted(OUT.iterdir()):
            rec.add_artifact(path)
        rec.add_artifact(RESULT)
        rec.add_artifact(RECORD)
        rec.add_metric("comparisons", len(rows), "configuration-cup pairs")
        for name, value in counts.items():
            rec.add_metric(name, value, "configuration-cup pairs")
        rec.add_metric("maximum_target_pixels", max(row["target_pixels"] for row in rows), "pixels")
        rec.complete(
            passed=True,
            criteria="All source configuration-cup pairs were compared and artifacts written; no perception or capability criterion.",
            review_status="pending_human_review", note="Candidate-model virtual raster comparison only.",
        )


if __name__ == "__main__":
    main()
