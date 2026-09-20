#!/usr/bin/env python3

from __future__ import annotations

import json
import unittest

from tools.cmake import REPOSITORY_ROOT

PRESETS_PATH = REPOSITORY_ROOT / "CMakePresets.json"


class MinimalPresetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(PRESETS_PATH.read_text(encoding="utf-8"))

    def preset(self, collection, name):
        return next(item for item in self.data[collection] if item["name"] == name)

    def test_schema_and_generator_support_cmake_320(self):
        self.assertEqual(self.data["version"], 2)
        configure = self.preset("configurePresets", "autonomy-minimal")
        self.assertEqual(configure["generator"], "Ninja")
        self.assertEqual(
            configure["binaryDir"],
            "${sourceDir}/build/autonomy-minimal",
        )

    def test_minimal_cache_enables_only_selected_domains(self):
        cache = self.preset("configurePresets", "autonomy-minimal")["cacheVariables"]
        expected_on = {
            "BUILD_TEST",
            "AUTONOMY_BUILD_COMMON",
            "AUTONOMY_BUILD_TRANSFORM",
            "AUTONOMY_BUILD_VEHICLE",
        }
        expected_off = {
            "BUILD_DOCS",
            "BUILD_TOOLS",
            "BUILD_GRPC",
            "BUILD_PROMETHEUS",
            "BUILD_GRID_MAP_DEMOS",
            "BUILD_AUTODRIVER",
            "BUILD_AUTOVIZ",
            "BUILD_AUTOSIM",
            "BUILD_ORBISVIEW",
            "BUILD_ONNXRUNTIME",
            "BUILD_TENSORRT",
            "BUILD_SHERPA_ONNX",
            "AUTOLINK_BUILD_PYTHON",
            "AUTOMSGS_BUILD_EXAMPLES",
            "AUTOMSGS_BUILD_TOOLS",
            "AUTOMSGS_BUILD_PYTHON",
            "AUTOMSGS_BUILD_TESTS",
            "AUTONOMY_BUILD_COMMON_OSQP",
            "AUTONOMY_BUILD_MAP",
            "AUTONOMY_BUILD_PREDICTION",
            "AUTONOMY_BUILD_CONTROL",
            "AUTONOMY_BUILD_PLANNING",
            "AUTONOMY_BUILD_PERCEPTION",
            "AUTONOMY_BUILD_LOCALIZATION",
            "AUTONOMY_BUILD_SENSOR",
            "AUTONOMY_BUILD_TASK",
            "AUTONOMY_BUILD_SYSTEM",
            "AUTONOMY_BUILD_AUDIO",
            "AUTONOMY_BUILD_BRIDGE",
            "AUTONOMY_BUILD_VISUALIZATION",
        }
        self.assertEqual(cache["CMAKE_BUILD_TYPE"], "Debug")
        self.assertEqual(cache["FORCE_DEBUG_BUILD"], "ON")
        self.assertEqual({name for name in expected_on if cache[name] == "ON"}, expected_on)
        self.assertEqual({name for name in expected_off if cache[name] == "OFF"}, expected_off)

    def test_build_and_test_presets_share_configure_preset(self):
        build = self.preset("buildPresets", "autonomy-minimal")
        test = self.preset("testPresets", "autonomy-minimal")
        self.assertEqual(build["configurePreset"], "autonomy-minimal")
        self.assertEqual(test["configurePreset"], "autonomy-minimal")
        self.assertTrue(test["output"]["outputOnFailure"])
        self.assertEqual(test["execution"]["noTestsAction"], "error")


if __name__ == "__main__":
    unittest.main()
