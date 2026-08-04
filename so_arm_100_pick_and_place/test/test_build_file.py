"""Validates build_file.py -- Sec 11 Phase 5's build-file/status-sidecar
loader. This is a compatible counterpart to the Blender addon's own
`so100_builder/io/build_file.py`, so most of these cases mirror that
module's own behavior directly, not a fresh interpretation of the prose
spec.

Run with:
    python3 -m unittest discover -s test
    colcon test --packages-select so_arm_100_pick_and_place

Deliberately plain unittest, matching so_arm_100_kinematics/test's and this
package's own test_stick_spec.py convention.
"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import so_arm_100_kinematics as sak
from so_arm_100_pick_and_place import build_file


def _stick(id_, order, base=(0.0, 0.0, 0.0), tip=(0.0, 0.0, 0.1)):
    return {
        "id": id_, "order": order, "base": list(base), "tip": list(tip),
        "roll_deg": 0.0, "length_m": 0.1, "shared_ends": 0, "supports": [],
        "validation": {"buildable": True, "reason": None}, "warnings": [],
    }


def _build_document(sticks, kinematics_version=None, frame="base_link"):
    return {
        "format": build_file.BUILD_FORMAT,
        "version": build_file.BUILD_VERSION,
        "generated": "2026-08-02T00:00:00Z",
        "source": "test.blend",
        "frame": frame,
        "units": "meters",
        "kinematics_version": kinematics_version if kinematics_version is not None else sak.__version__,
        "stock": {"section_m": [0.006, 0.006], "joint_allowance_m": 0.00325,
                   "length_range_m": [0.08, 0.15]},
        "build_volume": {"min": [-0.12, -0.45, 0.0], "max": [0.12, -0.29, 0.20]},
        "sticks": sticks,
    }


class TestStatusPathFor(unittest.TestCase):
    def test_replaces_the_json_suffix(self):
        self.assertEqual(build_file.status_path_for("tower_v3.build.json"), "tower_v3.build.status.json")

    def test_appends_when_there_is_no_json_suffix(self):
        self.assertEqual(build_file.status_path_for("tower_v3.build"), "tower_v3.build.status.json")


class TestLoadBuildFile(unittest.TestCase):
    def test_accepts_a_well_formed_file(self):
        doc = _build_document([_stick("s_001", 0)])
        parsed = build_file.load_build_file(json.dumps(doc))
        self.assertEqual(parsed["sticks"][0]["id"], "s_001")

    def test_rejects_not_json(self):
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file("not json")

    def test_rejects_wrong_format(self):
        doc = _build_document([_stick("s_001", 0)])
        doc["format"] = "something_else"
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file(json.dumps(doc))

    def test_rejects_wrong_version(self):
        doc = _build_document([_stick("s_001", 0)])
        doc["version"] = 999
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file(json.dumps(doc))

    def test_rejects_non_dense_order(self):
        doc = _build_document([_stick("s_001", 0), _stick("s_002", 5)])
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file(json.dumps(doc))

    def test_rejects_kinematics_version_mismatch(self):
        doc = _build_document([_stick("s_001", 0)], kinematics_version="0.0.1-definitely-not-real")
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file(json.dumps(doc))

    def test_rejects_wrong_frame(self):
        doc = _build_document([_stick("s_001", 0)], frame="some_other_frame")
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_build_file(json.dumps(doc))


class TestStatusDocument(unittest.TestCase):
    def test_omits_pending_entries(self):
        statuses = {
            "s_001": {"status": build_file.STATUS_PLACED, "at": "2026-08-02T00:00:00Z"},
            "s_002": {"status": build_file.STATUS_PENDING},
        }
        doc = build_file.status_document("x.build.json", statuses, current_index=1)
        self.assertIn("s_001", doc["sticks"])
        self.assertNotIn("s_002", doc["sticks"])

    def test_includes_reason_when_present(self):
        statuses = {"s_001": {"status": build_file.STATUS_FAILED, "reason": "grasp verification failed"}}
        doc = build_file.status_document("x.build.json", statuses)
        self.assertEqual(doc["sticks"]["s_001"]["reason"], "grasp verification failed")


class TestLoadStatusFile(unittest.TestCase):
    def test_round_trips_a_written_document(self):
        statuses = {"s_001": {"status": build_file.STATUS_PLACED, "at": "2026-08-02T00:00:00Z"}}
        doc = build_file.status_document("x.build.json", statuses, current_index=1)
        current_index, entries = build_file.load_status_file(json.dumps(doc))
        self.assertEqual(current_index, 1)
        self.assertEqual(entries["s_001"]["status"], build_file.STATUS_PLACED)

    def test_unknown_status_degrades_to_pending(self):
        doc = {
            "format": build_file.STATUS_FORMAT, "version": build_file.STATUS_VERSION,
            "build_file": "x.build.json", "updated": "2026-08-02T00:00:00Z", "current_index": 0,
            "sticks": {"s_001": {"status": "some_future_status", "at": ""}},
        }
        _current_index, entries = build_file.load_status_file(json.dumps(doc))
        self.assertEqual(entries["s_001"]["status"], build_file.STATUS_PENDING)

    def test_rejects_wrong_format(self):
        with self.assertRaises(build_file.ProtocolError):
            build_file.load_status_file(json.dumps({"format": "not_a_status_sidecar"}))


class TestNextStickToLoad(unittest.TestCase):
    def test_returns_the_first_non_placed_stick(self):
        ordered = ["s_001", "s_002", "s_003"]
        statuses = {"s_001": {"status": build_file.STATUS_PLACED}}
        self.assertEqual(build_file.next_stick_to_load(ordered, statuses), "s_002")

    def test_does_not_skip_a_failed_stick(self):
        ordered = ["s_001", "s_002"]
        statuses = {"s_001": {"status": build_file.STATUS_FAILED}}
        self.assertEqual(build_file.next_stick_to_load(ordered, statuses), "s_001")

    def test_returns_none_when_everything_is_placed(self):
        ordered = ["s_001", "s_002"]
        statuses = {"s_001": {"status": build_file.STATUS_PLACED}, "s_002": {"status": build_file.STATUS_PLACED}}
        self.assertIsNone(build_file.next_stick_to_load(ordered, statuses))


if __name__ == "__main__":
    unittest.main()
