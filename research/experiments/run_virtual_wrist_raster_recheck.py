"""Research-only all-pair virtual raster reproducibility recheck; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260916-004-virtual-wrist-raster-recheck"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260916-004-virtual-wrist-raster-recheck.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
SEGMENT = ROOT / "research/artifacts/EXP-20260902-002-virtual-wrist-segment-contact-scan/segment-contact-scan.json"
SOURCE = ROOT / "research/artifacts/EXP-20260902-003-virtual-wrist-raster-visibility/raster-visibility.json"
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
    source = json.loads(SOURCE.read_text())
    source_rows = {(row["configuration_index"], row["cup_index"]): row for row in source["rows"]}
    expected_pairs = len(segment["rows"]) * len(segment["cups"])
    if len(source_rows) != expected_pairs:
        raise ValueError("source raster table does not contain one unique row per configuration-cup pair")
    rec = RunRecorder(
        run_id=RID,
        objective="Recheck candidate-model wrist segmentation rows with a target-placement invariant and without attributing discrepancies.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_segment_table", "path": SEGMENT.relative_to(ROOT).as_posix(), "sha256": file_hash(SEGMENT)},
            {"kind": "source_raster_table", "path": SOURCE.relative_to(ROOT).as_posix(), "sha256": file_hash(SOURCE)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994"},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260902-003 raster table is retained as the source comparison, not assumed reproducible."},
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
        cup.rgba = [.1, .5, .9, 1]
        model = spec.compile()
        data = mujoco.MjData(model)
        cup_id = model.geom("virtual_cup").id
        renderer = mujoco.Renderer(model, height=cfg["protocol"]["renderer"]["height"], width=cfg["protocol"]["renderer"]["width"])
        renderer.enable_segmentation_rendering()
        rows = []
        try:
            for segment_row in segment["rows"]:
                for cup_index, position in enumerate(segment["cups"]):
                    expected_position = np.asarray(position, dtype=float)
                    data.qpos[:] = segment_row["qpos"]
                    model.geom_pos[cup_id] = expected_position
                    mujoco.mj_forward(model, data)
                    if not np.allclose(data.geom_xpos[cup_id], expected_position, rtol=0.0, atol=1e-12):
                        raise RuntimeError(f"virtual_cup position invariant failed for configuration={segment_row['sample_index']} cup={cup_index}")
                    renderer.update_scene(data, camera=cfg["protocol"]["camera"])
                    segmentation = renderer.render()
                    target_pixels = int(np.count_nonzero(
                        (segmentation[:, :, 0] == cup_id) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
                    ))
                    current_visible = target_pixels > 0
                    prior = source_rows[(segment_row["sample_index"], cup_index)]
                    rows.append({
                        "configuration_index": segment_row["sample_index"], "cup_index": cup_index,
                        "current_target_pixels": target_pixels, "current_raster_visible": current_visible,
                        "source_target_pixels": prior["target_pixels"], "source_raster_visible": prior["raster_visible"],
                        "same_raster_visibility": current_visible == prior["raster_visible"],
                    })
        finally:
            renderer.close()
        counts = {
            "source_and_current_visible": sum(row["source_raster_visible"] and row["current_raster_visible"] for row in rows),
            "source_invisible_current_visible": sum(not row["source_raster_visible"] and row["current_raster_visible"] for row in rows),
            "source_visible_current_invisible": sum(row["source_raster_visible"] and not row["current_raster_visible"] for row in rows),
            "source_and_current_invisible": sum(not row["source_raster_visible"] and not row["current_raster_visible"] for row in rows),
            "same_raster_visibility": sum(row["same_raster_visibility"] for row in rows),
        }
        meta = {"run_id": RID, "mujoco": mujoco.__version__, "source_run": source["run_id"], "placement_invariant": "passed for every row", "rows": rows, "counts": counts}
        (OUT / "raster-recheck.json").write_bytes(strict_json_bytes(meta))
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {"comparisons": len(rows), "placement_invariant_samples": len(rows), **counts},
            "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_wrist_raster_recheck",
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
            criteria="All source pairs satisfied target placement and were rechecked; agreement is an observation, not a capability criterion.",
            review_status="pending_human_review", note="Candidate-model raster reproducibility recheck only.",
        )


if __name__ == "__main__":
    main()
