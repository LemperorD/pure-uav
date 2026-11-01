"""CLI failures must leave diagnoses, avoid overwrites, and return failure codes."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class CliTests(unittest.TestCase):
    def test_missing_input_and_nonempty_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            out = root/"result"
            cmd = [sys.executable, str(ROOT/"calibrate_extrinsics.py"), "--camera", str(root/"missing.yaml"),
                   "--output", str(out)]
            first = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(first.returncode, 2)
            report = out/"report.json"
            before = report.read_bytes()
            self.assertEqual(json.loads(before)["status"], "failed")
            self.assertFalse((out/"camchain-kalibr.yaml").exists())
            second = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(second.returncode, 2)
            self.assertIn("非空", second.stderr)
            self.assertEqual(report.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
