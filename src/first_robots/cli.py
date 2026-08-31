"""Minimal product entrypoint for the planning-stage project."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass


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
        stage="planning",
        research="optional_pending",
        runtime_capabilities=("project_status",),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="First-Robots project entrypoint")
    parser.add_argument("command", nargs="?", choices=("status",), default="status")
    return parser


def main(argv: list[str] | None = None) -> int:
    build_parser().parse_args(argv)
    print(json.dumps(asdict(current_status()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
