import json
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from xcore_sdk_python.raw_recorder import RawEpisodeRecorder, RecordingError


class RawEpisodeRecorderTest(unittest.TestCase):
    def make_recorder(self, root: Path) -> RawEpisodeRecorder:
        return RawEpisodeRecorder(
            root,
            control_hz=100.0,
            joint_signs=[1, 1, -1, -1, 1, 1],
            task="test task",
            session_time=datetime(2026, 8, 26, 12, 0, tzinfo=timezone.utc),
        )

    def test_save_episode_writes_manifest_and_jsonl(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = self.make_recorder(Path(directory))
            partial = recorder.start_episode()
            recorder.add_sample({"action": [0.0] * 7, "joint_positions": [0.1] * 7})
            recorder.add_sample({"action": [0.2] * 7, "joint_positions": [0.3] * 7})
            final = recorder.save_episode()

            self.assertFalse(partial.exists())
            self.assertEqual(final.name, "episode_000000.jsonl")
            rows = [json.loads(line) for line in final.read_text().splitlines()]
            self.assertEqual([row["sequence"] for row in rows], [0, 1])
            manifest = json.loads((recorder.session_dir / "manifest.json").read_text())
            self.assertEqual(manifest["format"], "xcore_cr7_gello_raw")
            self.assertEqual(manifest["format_version"], 1)
            self.assertEqual(manifest["task"], "test task")

    def test_discard_removes_partial_and_reuses_episode_index(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = self.make_recorder(Path(directory))
            partial = recorder.start_episode()
            recorder.add_sample({"value": 1})
            recorder.discard_episode()

            self.assertFalse(partial.exists())
            self.assertEqual(recorder.episode_index, 0)
            self.assertEqual(
                recorder.start_episode().name, "episode_000000.jsonl.partial"
            )
            recorder.close_interrupted()

    def test_interrupted_episode_keeps_partial_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = self.make_recorder(Path(directory))
            partial = recorder.start_episode()
            recorder.add_sample({"value": 1})

            result = recorder.close_interrupted()

            self.assertEqual(result, partial)
            self.assertTrue(partial.exists())
            self.assertEqual(json.loads(partial.read_text())["sequence"], 0)

    def test_empty_episode_is_not_saved(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            recorder = self.make_recorder(Path(directory))
            partial = recorder.start_episode()

            with self.assertRaisesRegex(RecordingError, "empty episode"):
                recorder.save_episode()

            self.assertFalse(partial.exists())

    def test_same_second_sessions_get_unique_directories(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            first = self.make_recorder(Path(directory))
            second = self.make_recorder(Path(directory))

            self.assertNotEqual(first.session_dir, second.session_dir)
            self.assertEqual(second.session_dir.name, "session_20260826_120000_01")


if __name__ == "__main__":
    unittest.main()
