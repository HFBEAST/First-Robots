"""Research-only association of recorded candidate-model results; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260916-005-virtual-wrist-projection-raster-association"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260916-005-virtual-wrist-projection-raster-association.json"
PROJECTION = ROOT / "research/artifacts/EXP-20260916-002-virtual-wrist-projection-diagnosis/projection-diagnosis.json"
RECHECK = ROOT / "research/artifacts/EXP-20260916-004-virtual-wrist-raster-recheck/raster-recheck.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def keyed(rows: list[dict]) -> dict[tuple[int, int], dict]:
    result = {(row["configuration_index"], row["cup_index"]): row for row in rows}
    if len(result) != len(rows):
        raise ValueError("source table has duplicate configuration-cup keys")
    return result


def main() -> None:
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    cfg = json.loads(CFG.read_text())
    projection = json.loads(PROJECTION.read_text())
    recheck = json.loads(RECHECK.read_text())
    projection_rows = keyed(projection["rows"])
    recheck_rows = keyed(recheck["rows"])
    if len(projection_rows) != cfg["protocol"]["expected_pairs"] or set(projection_rows) != set(recheck_rows):
        raise ValueError("source tables do not form the expected one-to-one join")
    rec = RunRecorder(
        run_id=RID,
        objective="Associate recorded candidate-model projection conditions with a raster recheck without asserting causality or hardware capability.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_projection_table", "path": PROJECTION.relative_to(ROOT).as_posix(), "sha256": file_hash(PROJECTION)},
            {"kind": "source_raster_recheck_table", "path": RECHECK.relative_to(ROOT).as_posix(), "sha256": file_hash(RECHECK)},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260902-003 historical all-zero table is not used as the current visibility source."},
        level="formal",
    )
    with rec:
        OUT.mkdir(parents=True)
        (OUT / "config.json").write_bytes(strict_json_bytes(cfg))
        rows = []
        for key in sorted(projection_rows):
            projection_row = projection_rows[key]
            recheck_row = recheck_rows[key]
            rows.append({
                "configuration_index": key[0], "cup_index": key[1],
                "current_raster_visible": recheck_row["current_raster_visible"],
                "current_target_pixels": recheck_row["current_target_pixels"],
                **{condition: projection_row[condition] for condition in cfg["protocol"]["conditions"]},
            })
        visible = [row for row in rows if row["current_raster_visible"]]
        invisible = [row for row in rows if not row["current_raster_visible"]]
        counts = {"current_raster_visible": len(visible), "current_raster_invisible": len(invisible)}
        for condition in cfg["protocol"]["conditions"]:
            counts[f"visible_{condition}"] = sum(row[condition] for row in visible)
            counts[f"invisible_{condition}"] = sum(row[condition] for row in invisible)
        meta = {"run_id": RID, "source_runs": [projection["run_id"], recheck["run_id"]], "rows": rows, "counts": counts}
        (OUT / "projection-raster-association.json").write_bytes(strict_json_bytes(meta))
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {"comparisons": len(rows), **counts}, "limits": cfg["non_claims"],
            "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_wrist_projection_raster_association",
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
        rec.complete(
            passed=True,
            criteria="All source keys joined exactly once and association artifacts were written; no causal or capability criterion.",
            review_status="pending_human_review", note="Candidate-model projection/raster association only.",
        )


if __name__ == "__main__":
    main()
