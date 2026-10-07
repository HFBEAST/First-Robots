"""Minimal product entrypoint for the planning-stage project."""

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
        runtime_capabilities=("project_status", "virtual_reach"),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="First-Robots project entrypoint")
    parser.add_argument("command", nargs="?", choices=("status", "reach"), default="status")
    parser.add_argument("--config", type=Path, default=Path("config/sim_reach.json"))
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "status":
        print(json.dumps(asdict(current_status()), ensure_ascii=False, indent=2))
        return 0
    from .simulation import simulate_reach

    configuration = json.loads(args.config.read_text(encoding="utf-8"))
    report, _ = simulate_reach(Path(configuration["model"]), configuration)
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report["reason"] == "motion_completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
