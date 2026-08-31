"""Publish append-only evidence for one pinned MRBTP-demo exploratory execution."""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from experiment_management.run import sha256_bytes, strict_json_bytes, write_json, write_json_pair


RUN_ID = "EXP-20260831-002-mrbtp-demo-exploratory"
ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_DIR = ROOT / "research" / "artifacts" / RUN_ID
RUN_PATH = ROOT / "research" / "runs" / f"{RUN_ID}.json"
RECORD_PATH = ROOT / "research" / "experiments" / f"{RUN_ID}.record.json"
RESULT_PATH = ROOT / "research" / "reports" / f"{RUN_ID}.result.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return f"sha256:{digest}"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def git(*args: str) -> str:
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def main() -> None:
    artifact_names = [
        "exit-code.txt",
        "source-head.txt",
        "source-status.txt",
        "uv-pip-freeze.txt",
        "verify.log",
    ]
    artifacts = []
    for name in artifact_names:
        path = ARTIFACT_DIR / name
        if not path.is_file():
            raise FileNotFoundError(path)
        artifacts.append({"path": path.relative_to(ROOT).as_posix(), "sha256": sha256(path)})

    result = {
        "schema_version": "first-robots/research-result/v1",
        "run_id": RUN_ID,
        "status": "observed_pending_human_review",
        "observations": {
            "process_exit_code": 0,
            "shared": {"attempts_used": 3, "attempt_outcomes": [False, False, True], "successful_steps": 31},
            "no_share": {"attempts_used": 3, "attempt_outcomes": [False, False, True], "successful_steps": 35},
            "runtime": {"python": "3.12.11", "pygame": "2.6.1", "environment_manager": "uv"},
            "collection_exception": "`python -m pip freeze` was unavailable because the uv-created environment has no pip module; the separately captured `uv pip freeze` artifact is the dependency record."
        },
        "interpretation": [
            "The pinned upstream self-check was executable in this local environment.",
            "The two initial failures in each mode and the third-attempt successes are retained in the log.",
            "This run does not establish a performance difference between shared and no-share because retries and the upstream exit criterion are not a controlled causal comparison.",
            "This run does not establish any MuJoCo, SO-101, camera, hardware, grasping, safety, or multi-robot-control capability for First-Robots."
        ],
        "review": {"status": "pending_human_review", "adopted": False}
    }
    record = {
        "schema_version": "first-robots/experiment-record/v1",
        "run_id": RUN_ID,
        "recorded_at": utc_now(),
        "kind": "exploratory_upstream_code_execution",
        "source": {
            "repository": "https://github.com/DIDS-EI/MRBTP-demo",
            "revision": "3824c1c62bf38c89094c37b77573711e639c1d04",
            "license": "MIT"
        },
        "result": {
            "path": RESULT_PATH.relative_to(ROOT).as_posix(),
            "sha256": sha256_bytes(strict_json_bytes(result))
        },
        "review": {"status": "pending_human_review"}
    }
    manifest = {
        "schema_version": "experiment-management/run-manifest/v1",
        "run_id": RUN_ID,
        "level": "exploratory",
        "started_at": "2026-08-31T14:37:00Z",
        "ended_at": utc_now(),
        "objective": "Execute the pinned MRBTP-demo upstream self-check and retain raw reproducibility evidence without inferring First-Robots capability.",
        "execution": {
            "script": "research/sources/mrbtp-demo/app/verify.py",
            "command": [
                "research/.venvs/mrbtp-demo-py312/Scripts/python.exe", "-m", "app.verify", "--both", "--agents", "2", "--max-steps", "400", "--seed", "6", "--attempts", "5"
            ],
            "parameters": {"agents": 2, "max_steps": 400, "base_seed": 6, "attempts": 5, "modes": ["share", "no-share"]}
        },
        "code": {
            "repository": str(ROOT),
            "commit": git("rev-parse", "HEAD"),
            "traceable": True,
            "dirty_at_start": True,
            "external_source_revision": "3824c1c62bf38c89094c37b77573711e639c1d04"
        },
        "inputs": [
            {"kind": "configuration", "path": "research/configs/EXP-20260831-002-mrbtp-demo-exploratory.json", "sha256": sha256(ROOT / "research/configs/EXP-20260831-002-mrbtp-demo-exploratory.json")},
            {"kind": "source", "path": "research/sources/mrbtp-demo", "revision": "3824c1c62bf38c89094c37b77573711e639c1d04", "license": "MIT"}
        ],
        "runtime": {"python": "3.12.11", "platform": platform.platform(), "random_seed": 6, "environment_manager": "uv"},
        "baseline": {"kind": "comparison", "reference": "The upstream `--no-share` ablation was run only to confirm callability; stochastic retries prevent causal performance interpretation."},
        "results": {
            "passed": True,
            "criteria": "The upstream process exited 0 after recording both modes; this is an execution criterion only.",
            "metrics": [
                {"name": "shared_attempts_used", "value": 3, "unit": "attempts"},
                {"name": "shared_successful_steps", "value": 31, "unit": "steps"},
                {"name": "no_share_attempts_used", "value": 3, "unit": "attempts"},
                {"name": "no_share_successful_steps", "value": 35, "unit": "steps"}
            ],
            "duration_seconds": 17.0
        },
        "artifacts": artifacts,
        "review": {"status": "pending_human_review", "note": "Validated execution evidence is not an adopted conclusion."}
    }
    write_json_pair(first_path=RECORD_PATH, first_content=record, second_path=RESULT_PATH, second_content=result)
    write_json(RUN_PATH, manifest)
    print(RUN_PATH.relative_to(ROOT).as_posix())


if __name__ == "__main__":
    main()
