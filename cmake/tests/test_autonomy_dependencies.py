#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import textwrap
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
CMAKE_MODULE_DIR = REPOSITORY_ROOT / "cmake"


class AutonomyDependenciesTest(unittest.TestCase):
    def collect(self, modules, **options):
        with tempfile.TemporaryDirectory() as directory:
            script = pathlib.Path(directory) / "collect.cmake"
            output = pathlib.Path(directory) / "groups.txt"
            assignments = "\n".join(
                f"set({name} {'ON' if value else 'OFF'})"
                for name, value in options.items()
            )
            script.write_text(
                textwrap.dedent(
                    f"""
                    list(APPEND CMAKE_MODULE_PATH "{CMAKE_MODULE_DIR.as_posix()}")
                    set(AUTONOMY_ENABLED_MODULES {' '.join(modules)})
                    set(BUILD_GRPC OFF)
                    set(BUILD_AUTOVIZ OFF)
                    set(BUILD_PROMETHEUS OFF)
                    set(BUILD_SHERPA_ONNX OFF)
                    set(BUILD_ONNXRUNTIME OFF)
                    set(BUILD_TENSORRT OFF)
                    set(AUTONOMY_BUILD_COMMON_OSQP OFF)
                    {assignments}
                    include(autonomy_find_dependencies)
                    autonomy_collect_required_package_groups(groups)
                    file(WRITE "{output.as_posix()}" "${{groups}}")
                    """
                ),
                encoding="utf-8",
            )
            result = subprocess.run(
                ["cmake", "-P", str(script)],
                check=False,
                capture_output=True,
                text=True,
            )
            value = output.read_text(encoding="utf-8") if output.exists() else ""
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return value

    def test_common_groups(self):
        self.assertEqual(
            self.collect(["common"]),
            "core;common_math",
        )

    def test_minimal_groups_do_not_add_domain_packages(self):
        self.assertEqual(
            self.collect(["common", "transform", "vehicle"]),
            "core;common_math",
        )

    def test_map_adds_map_group(self):
        self.assertEqual(
            self.collect(["common", "transform", "map"]),
            "core;common_math;common_vision;map",
        )

    def test_localization_adds_localization_group(self):
        self.assertEqual(
            self.collect(["common", "transform", "localization"]),
            "core;common_math;common_vision;localization",
        )

    def test_task_stack_adds_domain_groups_once(self):
        self.assertEqual(
            self.collect(["common", "transform", "map", "control", "task"]),
            "core;common_math;common_vision;map;control;task",
        )

    def test_companion_groups_follow_options_and_domains(self):
        self.assertEqual(
            self.collect(
                ["common", "transform", "map", "control", "task", "system", "bridge"],
                BUILD_GRPC=True,
                BUILD_PROMETHEUS=True,
            ),
            "core;common_math;common_vision;map;control;task;grpc;prometheus",
        )
        self.assertEqual(
            self.collect(
                ["common", "audio"],
                BUILD_SHERPA_ONNX=True,
            ),
            "core;common_math;sherpa_onnx",
        )
        self.assertEqual(
            self.collect(["common"], BUILD_AUTOVIZ=True),
            "core;common_math;autoviz",
        )

    def test_inference_group_requires_option_and_consumer(self):
        self.assertEqual(
            self.collect(["common"], BUILD_ONNXRUNTIME=True),
            "core;common_math;common_vision;inference",
        )
        self.assertNotIn(
            "inference",
            self.collect(["transform"], BUILD_TENSORRT=True),
        )

    def test_visualization_adds_foxglove_group(self):
        self.assertEqual(
            self.collect(["common", "transform", "map", "visualization"]),
            "core;common_math;common_vision;map;foxglove",
        )

    def test_osqp_group_is_controlled_by_common_option(self):
        self.assertEqual(
            self.collect(["common"], AUTONOMY_BUILD_COMMON_OSQP=False),
            "core;common_math",
        )
        self.assertEqual(
            self.collect(["common"], AUTONOMY_BUILD_COMMON_OSQP=True),
            "core;common_math;osqp",
        )


if __name__ == "__main__":
    unittest.main()
