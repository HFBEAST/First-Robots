"""Research-only single-sample virtual geometry diagnostic; never controls hardware."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from experiment_management.run import RunRecorder, sha256_bytes, strict_json_bytes, write_json_pair

RID = "EXP-20260916-003-virtual-wrist-local-geometry-diagnostic"
ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "research/configs/EXP-20260916-003-virtual-wrist-local-geometry-diagnostic.json"
SCENE = ROOT / "research/sources/mujoco_menagerie_robotstudio_so101/robotstudio_so101/scene.xml"
SEGMENT = ROOT / "research/artifacts/EXP-20260902-002-virtual-wrist-segment-contact-scan/segment-contact-scan.json"
PROJECTION = ROOT / "research/artifacts/EXP-20260916-002-virtual-wrist-projection-diagnosis/projection-diagnosis.json"
OUT = ROOT / "research/artifacts" / RID
RESULT = ROOT / "research/reports" / f"{RID}.result.json"
RECORD = ROOT / "research/experiments" / f"{RID}.record.json"


def file_hash(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def signed_cylinder_distance(radial: float, axial: float, radius: float, half_height: float) -> float:
    delta = np.array([radial - radius, axial - half_height])
    return float(np.linalg.norm(np.maximum(delta, 0.0)) + min(float(np.max(delta)), 0.0))


def main() -> None:
    if OUT.exists() or RESULT.exists() or RECORD.exists():
        raise FileExistsError("refusing overwrite")
    cfg = json.loads(CFG.read_text())
    segment = json.loads(SEGMENT.read_text())
    projection = json.loads(PROJECTION.read_text())
    selected = [row for row in projection["rows"] if row["ray_visible"] and row["inside_candidate_frustum_and_clip"] and not row["raster_visible"]]
    if len(selected) != 1:
        raise ValueError(f"expected exactly one unresolved source row, got {len(selected)}")
    selected = selected[0]
    qpos = next(row["qpos"] for row in segment["rows"] if row["sample_index"] == selected["configuration_index"])
    expected_position = np.asarray(segment["cups"][selected["cup_index"]], dtype=float)
    rec = RunRecorder(
        run_id=RID,
        objective="Record one unresolved candidate-model camera/target geometry case without asserting a visibility cause or hardware capability.",
        script=Path(__file__), repo_root=ROOT, output_root=ROOT / "research/runs",
        parameters=cfg["protocol"], random_seed=11,
        inputs=[
            {"kind": "configuration", "path": CFG.relative_to(ROOT).as_posix(), "sha256": file_hash(CFG)},
            {"kind": "source_segment_table", "path": SEGMENT.relative_to(ROOT).as_posix(), "sha256": file_hash(SEGMENT)},
            {"kind": "source_projection_table", "path": PROJECTION.relative_to(ROOT).as_posix(), "sha256": file_hash(PROJECTION)},
            {"kind": "model", "path": SCENE.relative_to(ROOT).as_posix(), "sha256": file_hash(SCENE), "revision": "da76818e269b82289eba39808e2fb91d679d6994"},
        ],
        baseline={"kind": "comparison", "reference": "EXP-20260916-002 leaves one center-in-frustum/raster-invisible pair unresolved."},
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
        data.qpos[:] = qpos
        model.geom_pos[cup_id] = expected_position
        mujoco.mj_forward(model, data)
        world_center = data.geom_xpos[cup_id].copy()
        if not np.allclose(world_center, expected_position, rtol=0.0, atol=1e-12):
            raise RuntimeError("virtual_cup position invariant failed")
        camera_id = model.camera(cfg["protocol"]["renderer"]["camera"]).id
        camera_position = data.cam_xpos[camera_id].copy()
        cup_to_world = data.geom_xmat[cup_id].reshape(3, 3)
        camera_in_cup = cup_to_world.T @ (camera_position - world_center)
        radial = float(np.linalg.norm(camera_in_cup[:2]))
        axial = abs(float(camera_in_cup[2]))
        radius, half_height = float(model.geom_size[cup_id][0]), float(model.geom_size[cup_id][1])
        inside_cylinder = radial <= radius and axial <= half_height
        signed_distance = signed_cylinder_distance(radial, axial, radius, half_height)
        width, height = cfg["protocol"]["renderer"]["width"], cfg["protocol"]["renderer"]["height"]
        rgb_renderer = mujoco.Renderer(model, height=height, width=width)
        depth_renderer = mujoco.Renderer(model, height=height, width=width)
        segmentation_renderer = mujoco.Renderer(model, height=height, width=width)
        try:
            rgb_renderer.update_scene(data, camera=cfg["protocol"]["renderer"]["camera"])
            Image.fromarray(rgb_renderer.render()).save(OUT / "wrist-rgb.png")
            depth_renderer.enable_depth_rendering()
            depth_renderer.update_scene(data, camera=cfg["protocol"]["renderer"]["camera"])
            depth = depth_renderer.render()
            np.save(OUT / "wrist-depth.npy", depth)
            segmentation_renderer.enable_segmentation_rendering()
            segmentation_renderer.update_scene(data, camera=cfg["protocol"]["renderer"]["camera"])
            segmentation = segmentation_renderer.render()
            np.save(OUT / "wrist-segmentation.npy", segmentation)
        finally:
            rgb_renderer.close()
            depth_renderer.close()
            segmentation_renderer.close()
        target_pixels = int(np.count_nonzero(
            (segmentation[:, :, 0] == cup_id) & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
        ))
        meta = {
            "run_id": RID, "mujoco": mujoco.__version__,
            "selection": {"configuration_index": selected["configuration_index"], "cup_index": selected["cup_index"],
                          "source_ray_visible": selected["ray_visible"], "source_raster_visible": selected["raster_visible"]},
            "geometry": {"input_position": expected_position.tolist(), "world_center": world_center.tolist(),
                         "camera_position": camera_position.tolist(), "camera_in_cup_coordinates": camera_in_cup.tolist(),
                         "radius": radius, "half_height": half_height, "radial_distance": radial, "axial_distance": axial,
                         "camera_center_inside_cylinder": inside_cylinder, "signed_distance_to_cylinder_surface": signed_distance},
            "render": {"target_pixels": target_pixels, "width": width, "height": height},
        }
        (OUT / "local-geometry.json").write_bytes(strict_json_bytes(meta))
        result = {
            "schema_version": "first-robots/research-result/v1", "run_id": RID,
            "status": "observed_pending_human_review",
            "observations": {"selected_configuration_index": selected["configuration_index"], "selected_cup_index": selected["cup_index"],
                             "camera_center_inside_cylinder": inside_cylinder, "signed_distance_to_cylinder_surface": signed_distance,
                             "rendered_target_pixels": target_pixels},
            "limits": cfg["non_claims"], "review": {"status": "pending_human_review", "adopted": False},
        }
        record = {
            "schema_version": "first-robots/experiment-record/v1", "run_id": RID,
            "kind": "formal_virtual_wrist_local_geometry_diagnostic",
            "result": {"path": RESULT.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(strict_json_bytes(result))},
            "review": {"status": "pending_human_review"},
        }
        write_json_pair(first_path=RECORD, first_content=record, second_path=RESULT, second_content=result)
        for path in sorted(OUT.iterdir()):
            rec.add_artifact(path)
        rec.add_artifact(RESULT)
        rec.add_artifact(RECORD)
        rec.add_metric("selected_pairs", 1, "configuration-cup pairs")
        rec.add_metric("camera_center_inside_cylinder", inside_cylinder, "boolean")
        rec.add_metric("signed_distance_to_cylinder_surface", signed_distance, "meters")
        rec.add_metric("rendered_target_pixels", target_pixels, "pixels")
        rec.complete(
            passed=True,
            criteria="The unique source pair satisfied placement and produced all diagnostic artifacts; no visibility or capability criterion.",
            review_status="pending_human_review", note="Candidate-model single-sample geometry diagnostic only.",
        )


if __name__ == "__main__":
    main()
