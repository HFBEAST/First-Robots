"""Product entrypoint for status and bounded virtual single-arm tasks."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProjectStatus:
    name: str
    stage: str
    research: str
    runtime_capabilities: tuple[str, ...]


def current_status() -> ProjectStatus:
    """Return only product state; never inspect the optional research sidecar."""
    return ProjectStatus(
        name="First-Robots",
        stage="single_arm_simulation",
        research="optional_active",
        runtime_capabilities=("project_status", "virtual_reach", "virtual_pick_place"),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="First-Robots project entrypoint")
    parser.add_argument("command", nargs="?", choices=("status", "reach", "pick-place"), default="status")
    parser.add_argument("--config", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "status":
        print(json.dumps(asdict(current_status()), ensure_ascii=False, indent=2))
        return 0
    config_path = args.config or Path("config/sim_pick_place.json" if args.command == "pick-place" else "config/sim_reach.json")
    configuration = json.loads(config_path.read_text(encoding="utf-8"))
    if args.command == "pick-place":
        from .grasping import simulate_pick_place

        report, _ = simulate_pick_place(Path(configuration["model"]), configuration)
        passed = report["completed"]
    else:
        from .simulation import simulate_reach

        report, _ = simulate_reach(Path(configuration["model"]), configuration)
        passed = report["reason"] == "motion_completed"
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
