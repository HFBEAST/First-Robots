"""Research-only discrete configuration scan; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260902-002-virtual-wrist-segment-contact-scan"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260902-002-virtual-wrist-segment-contact-scan.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
COVERAGE = ROOT / "research/artifacts/EXP-20260901-001-virtual-wrist-coverage/coverage.json"
CONTACTS = ROOT / "research/artifacts/EXP-20260901-002-virtual-wrist-contact-gate/contact-coverage.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def visible(model: mujoco.MjModel, data: mujoco.MjData, cup_id: int) -> bool:
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
    coverage = json.loads(COVERAGE.read_text())
    contacts = json.loads(CONTACTS.read_text())
    start = next(row for row in coverage["pose_results"] if row["sample_index"] == 0)
    end = contacts["first_maximum_zero_contact"]
    if len(coverage["cups"]) != len(cfg["protocol"]["cup_seeds"]):
        raise ValueError("coverage cup count differs from configured cup seeds")
    rec = RunRecorder(
        run_id=RID,
        objective="Record a discrete candidate-model configuration segment without asserting a safe or executable motion.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_coverage_table", "path": COVERAGE.relative_to(ROOT).as_posix(), "sha256": file_hash(COVERAGE)},
            {"kind": "source_contact_table", "path": CONTACTS.relative_to(ROOT).as_posix(), "sha256": file_hash(CONTACTS)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994"},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260901-002 records static sampled poses, not intermediate configurations."},
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
        start_qpos = np.asarray(start["qpos"], dtype=float)
        end_qpos = np.asarray(end["qpos"], dtype=float)
        if start_qpos.shape != (model.nq,) or end_qpos.shape != (model.nq,):
            raise ValueError("source qpos does not match candidate model")
        rows = []
        for index, alpha in enumerate(np.linspace(0.0, 1.0, cfg["protocol"]["segment_samples"])):
            qpos = (1.0 - alpha) * start_qpos + alpha * end_qpos
            in_limits = bool(np.all(qpos >= model.jnt_range[:, 0]) and np.all(qpos <= model.jnt_range[:, 1]))
            contact_counts, visible_cups = [], 0
            for position in coverage["cups"]:
                data.qpos[:] = qpos
                model.geom_pos[cup_id] = position
                mujoco.mj_forward(model, data)
                contact_counts.append(int(data.ncon))
                visible_cups += int(visible(model, data, cup_id))
            rows.append({
                "sample_index": index, "alpha": float(alpha), "qpos": qpos.tolist(),
                "in_candidate_joint_limits": in_limits, "contact_counts": contact_counts,
                "max_contacts": max(contact_counts), "visible_cups": visible_cups,
            })
        meta = {
            "run_id": RID, "mujoco": mujoco.__version__, "source_endpoint_sample_index": end["sample_index"],
            "source_endpoint_visible_cups": end["visible_cups"], "cups": coverage["cups"], "rows": rows,
        }
        (OUT / "segment-contact-scan.json").write_bytes(strict_json_bytes(meta))
        zero_contact_samples = sum(row["max_contacts"] == 0 for row in rows)
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {
                "segment_samples": len(rows), "source_endpoint_sample_index": end["sample_index"],
                "in_candidate_joint_limits_samples": sum(row["in_candidate_joint_limits"] for row in rows),
                "zero_contact_all_cups_samples": zero_contact_samples,
                "maximum_visible_cups": max(row["visible_cups"] for row in rows),
            },
            "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_discrete_configuration_segment_scan",
            "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
            "review": {"status": "pending_human_review"},
        }
        write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)
        for path in sorted(OUT.iterdir()):
            rec.add_artifact(path)
        rec.add_artifact(RESULT)
        rec.add_artifact(RECORD)
        rec.add_metric("segment_samples", len(rows), "configurations")
        rec.add_metric("in_candidate_joint_limits_samples", sum(row["in_candidate_joint_limits"] for row in rows), "configurations")
        rec.add_metric("zero_contact_all_cups_samples", zero_contact_samples, "configurations")
        rec.add_metric("maximum_visible_cups", max(row["visible_cups"] for row in rows), "cups")
        rec.complete(
            passed=True,
            criteria="All fixed discrete configurations and cup positions were recorded; no motion or safety criterion.",
            review_status="pending_human_review", note="Candidate-model discrete configuration scan only.",
        )


if __name__ == "__main__":
    main()
