from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from first_robots.cli import current_status  # noqa: E402


class ProjectStatusTests(unittest.TestCase):
    def test_planning_status_is_explicit_and_minimal(self) -> None:
        status = current_status()

        self.assertEqual(status.name, "First-Robots")
        self.assertEqual(status.stage, "planning")
        self.assertEqual(status.research, "optional_pending")
        self.assertEqual(status.runtime_capabilities, ("project_status",))


if __name__ == "__main__":
    unittest.main()
