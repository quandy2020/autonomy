#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import textwrap
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]


class MinimalTargetBoundariesTest(unittest.TestCase):
    def test_minimal_profile_has_explicit_export_safe_target_boundaries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            build = root / "build"
            report = root / "target-properties.txt"
            probe = root / "probe.cmake"
            probe.write_text(
                textwrap.dedent(
                    f"""
                    function(_autonomy_write_target_report)
                      file(WRITE "{report.as_posix()}" "")
                      foreach(_target IN ITEMS
                          autonomy_proto autonomy_common autonomy_transform autonomy_vehicle
                          autonomy_common_test_support autonomy_transform_test_support
                          autonomy.common.endian_test autonomy.transform.static_transform_test)
                        if(NOT TARGET ${{_target}})
                          file(APPEND "{report.as_posix()}"
                            "${{_target}}|MISSING|MISSING|MISSING\n")
                          continue()
                        endif()
                        get_target_property(_target_type ${{_target}} TYPE)
                        if(_target_type STREQUAL "EXECUTABLE" OR
                            (_target MATCHES "_test_support$" AND
                             NOT _target_type STREQUAL "INTERFACE_LIBRARY"))
                          get_target_property(_links ${{_target}} LINK_LIBRARIES)
                        else()
                          get_target_property(_links ${{_target}} INTERFACE_LINK_LIBRARIES)
                        endif()
                        if(_links MATCHES "-NOTFOUND$")
                          set(_links "")
                        endif()
                        get_target_property(_includes ${{_target}} INTERFACE_INCLUDE_DIRECTORIES)
                        file(APPEND "{report.as_posix()}"
                          "${{_target}}|${{_target_type}}|${{_links}}|${{_includes}}\n")
                      endforeach()
                    endfunction()
                    cmake_language(DEFER CALL _autonomy_write_target_report)
                    """
                ),
                encoding="utf-8",
            )
            command = [
                "cmake",
                "-S",
                str(REPOSITORY_ROOT),
                "-B",
                str(build),
                "-G",
                "Ninja",
                "-DCMAKE_BUILD_TYPE=Debug",
                "-DFORCE_DEBUG_BUILD=ON",
                "-DBUILD_TEST=ON",
                "-DBUILD_DOCS=OFF",
                "-DBUILD_TOOLS=OFF",
                "-DBUILD_GRPC=OFF",
                "-DBUILD_PROMETHEUS=OFF",
                "-DBUILD_GRID_MAP_DEMOS=OFF",
                "-DBUILD_AUTODRIVER=OFF",
                "-DBUILD_AUTOVIZ=OFF",
                "-DBUILD_AUTOSIM=OFF",
                "-DBUILD_ONNXRUNTIME=OFF",
                "-DBUILD_TENSORRT=OFF",
                "-DBUILD_SHERPA_ONNX=OFF",
                "-DAUTONOMY_BUILD_COMMON=ON",
                "-DAUTONOMY_BUILD_COMMON_OSQP=OFF",
                "-DAUTONOMY_BUILD_TRANSFORM=ON",
                "-DAUTONOMY_BUILD_VEHICLE=ON",
                "-DAUTONOMY_BUILD_MAP=OFF",
                "-DAUTONOMY_BUILD_PREDICTION=OFF",
                "-DAUTONOMY_BUILD_CONTROL=OFF",
                "-DAUTONOMY_BUILD_PLANNING=OFF",
                "-DAUTONOMY_BUILD_PERCEPTION=OFF",
                "-DAUTONOMY_BUILD_LOCALIZATION=OFF",
                "-DAUTONOMY_BUILD_SENSOR=OFF",
                "-DAUTONOMY_BUILD_TASK=OFF",
                "-DAUTONOMY_BUILD_SYSTEM=OFF",
                "-DAUTONOMY_BUILD_AUDIO=OFF",
                "-DAUTONOMY_BUILD_BRIDGE=OFF",
                "-DAUTONOMY_BUILD_VISUALIZATION=OFF",
                f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={probe}",
            ]
            result = subprocess.run(command, check=False, capture_output=True, text=True)
            output = result.stdout + result.stderr
            self.assertEqual(result.returncode, 0, output)
            self.assertNotIn("Cannot determine link language for target", output)
            rows = {}
            for line in report.read_text(encoding="utf-8").splitlines():
                target, target_type, links, includes = line.split("|", 3)
                rows[target] = (target_type, links, includes)
            self.assertEqual(
                set(rows),
                {
                    "autonomy_proto",
                    "autonomy_common",
                    "autonomy_transform",
                    "autonomy_vehicle",
                    "autonomy_common_test_support",
                    "autonomy_transform_test_support",
                    "autonomy.common.endian_test",
                    "autonomy.transform.static_transform_test",
                },
            )
            for target, (target_type, links, includes) in rows.items():
                self.assertNotEqual(target_type, "MISSING", target)
                self.assertNotIn("TBB::tbb", links, target)
                for item in includes.split(";"):
                    if str(REPOSITORY_ROOT) in item:
                        self.assertTrue(item.startswith("$<BUILD_INTERFACE:"), (target, item))
            proto_links = rows["autonomy_proto"][1]
            self.assertIn("protobuf::libprotobuf", proto_links)
            self.assertIn("automsgs", proto_links)
            for forbidden in ("Ceres", "opencv", "yaml-cpp", "TBB"):
                self.assertNotIn(forbidden, proto_links)
            self.assertIn("gflags::gflags", rows["autonomy_common"][1])
            self.assertNotIn("opencv", rows["autonomy_common"][1].lower())
            self.assertIn("autonomy_common", rows["autonomy_transform"][1])
            self.assertIn("autonomy_common", rows["autonomy_vehicle"][1])
            self.assertIn("autonomy_proto", rows["autonomy_transform"][1])
            self.assertIn("autonomy_proto", rows["autonomy_vehicle"][1])

            for support_target in (
                "autonomy_common_test_support",
                "autonomy_transform_test_support",
            ):
                target_type, links, _ = rows[support_target]
                self.assertEqual(target_type, "INTERFACE_LIBRARY", support_target)
                self.assertEqual(links, "", support_target)

            common_test_links = rows["autonomy.common.endian_test"][1]
            common_test_link_items = common_test_links.split(";")
            self.assertIn("autonomy_common", common_test_link_items)
            self.assertIn("autonomy_common_test_support", common_test_link_items)
            for forbidden in ("autonomy_transform", "autonomy_vehicle", "autonomy"):
                self.assertNotIn(forbidden, common_test_link_items)

            transform_test_links = rows["autonomy.transform.static_transform_test"][1]
            transform_test_link_items = transform_test_links.split(";")
            self.assertIn("autonomy_transform", transform_test_link_items)
            self.assertIn("autonomy_transform_test_support", transform_test_link_items)
            self.assertNotIn("autonomy_vehicle", transform_test_link_items)
            self.assertNotIn("autonomy", transform_test_link_items)


if __name__ == "__main__":
    unittest.main()
