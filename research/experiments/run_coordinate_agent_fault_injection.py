"""Fault-inject Coordinator messages; this has no perception or robot execution path."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair
from research.protocols.coordinate_transfer_v2 import coordinate_agent_accepts, mock_execute

RID = "EXP-20260921-018-coordinate-agent-fault-injection"
CFG = ROOT / "research/configs" / f"{RID}.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
EXECUTION_RECORD = ROOT / "research/experiments" / f"{RID}.execution.record.json"


def sha256_file(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def coordinate_message(frame_id: str, scenario_version: str, point_world_m: np.ndarray) -> dict:
    return {"agent":"coordinate","source_message_agent":"detection","frame_id":frame_id,"scenario_version":scenario_version,"point_world_m":point_world_m.tolist()}


def rejected_case(messages: list[dict], frames: set[str], version: str, initial_state: list[float], target: np.ndarray) -> dict:
    accepted, reason, _ = coordinate_agent_accepts(messages, frames, version)
    state = {"cup_center_m":initial_state[:]}; executed, execution_reason = mock_execute(state, accepted, target)
    return {"coordinator_accepted":accepted,"reason":reason,"execution_agent":{"executed":executed,"reason":execution_reason},"state_unchanged":state["cup_center_m"] == initial_state}


def main() -> None:
    if OUT.exists() or RESULT.exists() or EXECUTION_RECORD.exists(): raise FileExistsError("refusing to overwrite formal artifacts, result, or execution record")
    config=json.loads(CFG.read_text(encoding="utf-8")); scenario=config["scenario"]; predecessor=ROOT/config["blocking_predecessor"]["result_path"]
    if sha256_file(predecessor) != config["blocking_predecessor"]["result_sha256"]: raise RuntimeError("frozen predecessor hash mismatch")
    recorder=RunRecorder(run_id=RID,objective="Verify Coordinator rejection of range-wide coordinate disagreement and stale-version messages before mock execution.",script=Path(__file__),repo_root=ROOT,output_root=ROOT/"research/runs",parameters=scenario,random_seed=None,inputs=[{"kind":"configuration","path":CFG.relative_to(ROOT).as_posix(),"sha256":sha256_file(CFG)},{"kind":"predecessor_result","path":predecessor.relative_to(ROOT).as_posix(),"sha256":sha256_file(predecessor)}],baseline={"kind":"coordinate_agent_protocol","reference":config["blocking_predecessor"]["run_id"]},level="formal")
    with recorder:
        OUT.mkdir(parents=True); (OUT/"config.json").write_bytes(strict_json_bytes(config)); frames=set(scenario["registered_frames"]); version=scenario["scenario_version"]; initial=scenario["cup_a_center_m"]; offset=np.asarray(scenario["coordinate_disagreement_offset_m"],dtype=float)
        targets=[np.array([x,y,scenario["target_b_grid_m"]["z"]],dtype=float) for x in scenario["target_b_grid_m"]["x"] for y in scenario["target_b_grid_m"]["y"]]; cases=[]
        for index,target in enumerate(targets,1):
            first=coordinate_message("fixed_depth_01",version,target); second=coordinate_message("fixed_depth_02",version,target); good_ok,good_reason,_=coordinate_agent_accepts([first,second],frames,version)
            disagreement=rejected_case([first,coordinate_message("fixed_depth_02",version,target+offset)],frames,version,initial,target)
            stale=rejected_case([first,coordinate_message("fixed_depth_02",scenario["wrong_version"],target)],frames,version,initial,target)
            cases.append({"case_id":f"B-{index:02d}","target_b_world_m":target.tolist(),"consistent_pair":{"coordinator_accepted":good_ok,"reason":good_reason},"coordinate_disagreement":disagreement,"stale_version":stale})
        passed=all(case["consistent_pair"]["coordinator_accepted"] and not case["coordinate_disagreement"]["coordinator_accepted"] and case["coordinate_disagreement"]["reason"]=="camera_world_coordinate_disagreement" and not case["coordinate_disagreement"]["execution_agent"]["executed"] and case["coordinate_disagreement"]["state_unchanged"] and not case["stale_version"]["coordinator_accepted"] and case["stale_version"]["reason"]=="scenario_version_mismatch" and not case["stale_version"]["execution_agent"]["executed"] and case["stale_version"]["state_unchanged"] for case in cases)
        observations={"barrier_absent":True,"target_count":len(targets),"coordinate_disagreement_offset_m":offset.tolist(),"cases":cases}; (OUT/"fault-injection-observations.json").write_bytes(strict_json_bytes(observations)); result={"schema_version":"first-robots/research-result/v1","run_id":RID,"status":"passed_pending_human_review" if passed else "blocked_pending_human_review","observations":observations,"limits":config["non_claims"],"review":{"status":"pending_human_review","adopted":False}}; record={"schema_version":"first-robots/experiment-record/v1","run_id":RID,"kind":"formal_coordinate_agent_fault_injection","result":{"path":RESULT.relative_to(ROOT).as_posix(),"sha256":sha256_bytes(strict_json_bytes(result))},"review":{"status":"pending_human_review","adopted":False}}
        write_json_pair(first_path=EXECUTION_RECORD,first_content=record,second_path=RESULT,second_content=result)
        for artifact in sorted(OUT.iterdir()): recorder.add_artifact(artifact)
        recorder.add_artifact(EXECUTION_RECORD); recorder.add_artifact(RESULT); recorder.add_metric("fault_injection_contract_valid",passed,"boolean"); recorder.add_metric("target_count",len(targets),"count"); recorder.complete(passed=passed,criteria="Every malformed coordinate/version message is rejected before mock execution while correct pairs accept.",review_status="pending_human_review",note="Coordinator-only virtual fault injection; not perception, calibration, robot execution, or safety evidence.")


if __name__ == "__main__": main()
