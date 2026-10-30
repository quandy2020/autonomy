#!/usr/bin/env python3
"""Tests for safe TUM RGB-D archive preparation."""

import io
import tarfile
import tempfile
import unittest
from pathlib import Path

from prepare_tum_rgbd import prepare


class PreparationTest(unittest.TestCase):
    @staticmethod
    def write_archive(path, sequence, member_name=None):
        payload = b"test data\n"
        with tarfile.open(path, "w:gz") as archive:
            member = tarfile.TarInfo(member_name or f"{sequence}/rgb.txt")
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))

    def test_extracts_without_removing_archive(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sequence = "rgbd_dataset_freiburg1_test"
            archive = root / f"{sequence}.tgz"
            self.write_archive(archive, sequence)
            results = prepare(root)
            self.assertEqual(results[0].status, "extracted")
            self.assertEqual((root / sequence / "rgb.txt").read_bytes(), b"test data\n")
            self.assertTrue(archive.is_file())

    def test_never_overwrites_existing_destination(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sequence = "rgbd_dataset_freiburg2_test"
            archive = root / f"{sequence}.tgz"
            self.write_archive(archive, sequence)
            destination = root / sequence
            destination.mkdir()
            marker = destination / "marker"
            marker.write_text("keep", encoding="utf-8")
            results = prepare(root)
            self.assertEqual(results[0].status, "already_present")
            self.assertEqual(marker.read_text(encoding="utf-8"), "keep")

    def test_rejects_path_traversal(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sequence = "rgbd_dataset_freiburg3_test"
            archive = root / f"{sequence}.tgz"
            self.write_archive(archive, sequence, f"{sequence}/../escape")
            results = prepare(root)
            self.assertEqual(results[0].status, "failed")
            self.assertFalse((root / sequence).exists())
            self.assertFalse((root / "escape").exists())


if __name__ == "__main__":
    unittest.main()
