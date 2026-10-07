from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from first_robots.cli import current_status  # noqa: E402


class ProjectStatusTests(unittest.TestCase):
    def test_simulation_stage_exposes_implemented_commands(self) -> None:
        status = current_status()

        self.assertEqual(status.name, "First-Robots")
        self.assertEqual(status.stage, "single_arm_simulation")
        self.assertEqual(status.research, "optional_active")
        self.assertEqual(status.runtime_capabilities, ("project_status", "virtual_reach", "virtual_pick_place"))


if __name__ == "__main__":
    unittest.main()
