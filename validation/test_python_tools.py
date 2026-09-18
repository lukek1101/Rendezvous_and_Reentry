"""Run from the project root: python -m unittest discover -s validation -p test_*.py"""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import DragDeorbitDesigner as deorbit
import J2PolarHohmannShooting as shooting
import mission_io


ROOT = Path(__file__).resolve().parents[1]


class ArchiveTests(unittest.TestCase):
    def test_existing_archive_hashes_are_preserved(self):
        for path in (ROOT / "configs/python_runs").glob("*.json"):
            config = json.loads(path.read_text(encoding="utf-8"))
            with self.subTest(case=path.name):
                self.assertEqual(mission_io.short_hash(shooting.config_settings_signature(config), 16),
                                 config["archive"]["settings_hash"])
                self.assertEqual(mission_io.short_hash(shooting.config_result_signature(config), 16),
                                 config["archive"]["result_hash"])

    def test_shared_exports_and_numpy_serialization(self):
        for module in (deorbit, shooting):
            self.assertIs(module.short_hash, mission_io.short_hash)
            self.assertIs(module.to_jsonable, mission_io.to_jsonable)
        value = {"array": np.array([1, 2]), "value": np.float64(0.5), "flag": np.bool_(True)}
        self.assertEqual(json.loads(mission_io.canonical_json_text(value)),
                         {"array": [1, 2], "value": 0.5, "flag": True})

    def test_upsert_keeps_other_cases_and_orders_by_time(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "index.json"
            newer = {"case_id": "b", "created_at": "2026-02-01"}
            older = {"case_id": "a", "created_at": "2026-01-01"}
            mission_io.update_case_index(path, newer, "test")
            mission_io.update_case_index(path, older, "test")
            mission_io.update_case_index(path, dict(newer, fuel=3), "test")
            index = json.loads(path.read_text())
            self.assertEqual([c["case_id"] for c in index["cases"]], ["a", "b"])
            self.assertEqual(index["cases"][1]["fuel"], 3)
            self.assertEqual(index["latest_case_id"], "b")

    def test_corrupt_index_is_not_silently_overwritten(self):
        for text in ('{broken', '[]', '{"cases":{}}', '{"cases":[3]}'):
            with self.subTest(text=text), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / "index.json"
                path.write_text(text)
                with self.assertRaises(ValueError):
                    mission_io.update_case_index(path, {"case_id": "new"}, "test")
                self.assertEqual(path.read_text(), text)

    def test_failed_serialization_keeps_existing_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "latest.json"
            path.write_text('{"previous":1}')
            with self.assertRaises(TypeError):
                mission_io.write_json_atomic(path, {"bad": object()})
            self.assertEqual(path.read_text(), '{"previous":1}')
            self.assertEqual(list(Path(folder).glob("*.tmp")), [])

    def test_deorbit_export_round_trip(self):
        config = json.loads((ROOT / "configs/latest_drag_deorbit_solution.json").read_text())
        # The stored mission fixture predates the designer's finite-burn
        # export schema. Supply the fields a current design run produces.
        config['maneuver'] = {'finite_burn_thrust_N': 300, 'finite_burn_isp_s': 200}
        config['phase3']['drag_deorbit'].update(burn_steering='VELOCITY_RETROGRADE',
                                               predicted_burn_duration_s=0)
        expected_hash = deorbit.attach_archive_metadata(config)['archive']['settings_hash']
        with tempfile.TemporaryDirectory() as folder:
            latest, archive = deorbit.write_outputs(config, Path(folder) / "latest.json")
            stored = json.loads(latest.read_text())
            self.assertEqual(stored, json.loads(archive.read_text()))
            self.assertEqual(stored["phase3"], config["phase3"])
            self.assertEqual(stored["archive"]["settings_hash"], expected_hash)
            index = json.loads((Path(folder) / "drag_deorbit_solution_index.json").read_text())
            self.assertEqual(len(index["cases"]), 1)
            self.assertTrue((Path(folder) / index["cases"][0]["path"]).is_file())


if __name__ == "__main__":
    unittest.main()
