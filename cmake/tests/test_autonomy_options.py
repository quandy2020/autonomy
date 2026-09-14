#!/usr/bin/env python3

import pathlib
import re
import subprocess
import tempfile
import textwrap
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
CMAKE_MODULE_DIR = REPOSITORY_ROOT / "cmake"
MODULES = (
    "common",
    "transform",
    "map",
    "vehicle",
    "prediction",
    "control",
    "planning",
    "perception",
    "localization",
    "sensor",
    "task",
    "system",
    "audio",
    "bridge",
    "visualization",
)


class AutonomyOptionsTest(unittest.TestCase):
    def configure(self, enabled=None):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "source"
            build = pathlib.Path(directory) / "build"
            source.mkdir()
            source.joinpath("CMakeLists.txt").write_text(
                textwrap.dedent(
                    f"""
                    cmake_minimum_required(VERSION 3.20)
                    project(autonomy_options_contract LANGUAGES NONE)
                    list(APPEND CMAKE_MODULE_PATH "{CMAKE_MODULE_DIR.as_posix()}")
                    set(AUTONOMY_MODULE_ORDER {' '.join(MODULES)})
                    include(autonomy_options)
                    autonomy_declare_module_options()
                    autonomy_validate_module_graph()
                    autonomy_compute_enabled_modules(AUTONOMY_ENABLED_MODULES)
                    file(WRITE "${{CMAKE_BINARY_DIR}}/enabled.txt"
                         "${{AUTONOMY_ENABLED_MODULES}}")
                    """
                ),
                encoding="utf-8",
            )
            command = ["cmake", "-S", str(source), "-B", str(build)]
            if enabled is not None:
                enabled = set(enabled)
                command.extend(
                    f"-DAUTONOMY_BUILD_{module.upper()}="
                    f"{'ON' if module in enabled else 'OFF'}"
                    for module in MODULES
                )
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
            )
            enabled_path = build / "enabled.txt"
            value = enabled_path.read_text(encoding="utf-8") if enabled_path.exists() else ""
            return result, value

    def test_defaults_enable_every_domain(self):
        result, enabled = self.configure()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(enabled, ";".join(MODULES))

    def test_minimal_enables_common_transform_vehicle(self):
        result, enabled = self.configure({"common", "transform", "vehicle"})
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(enabled, "common;transform;vehicle")

    def test_transform_without_common_is_rejected(self):
        result, _ = self.configure({"transform"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "AUTONOMY_BUILD_TRANSFORM=ON requires AUTONOMY_BUILD_COMMON=ON",
            result.stdout + result.stderr,
        )

    def test_control_without_map_is_rejected(self):
        result, _ = self.configure({"common", "transform", "control"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "AUTONOMY_BUILD_CONTROL=ON requires AUTONOMY_BUILD_MAP=ON",
            result.stdout + result.stderr,
        )

    def test_bridge_without_system_is_rejected(self):
        result, _ = self.configure({"common", "bridge"})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "AUTONOMY_BUILD_BRIDGE=ON requires AUTONOMY_BUILD_SYSTEM=ON",
            result.stdout + result.stderr,
        )

    def test_root_selects_modules_before_finding_dependencies(self):
        super_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_superproject.cmake"
        ).read_text(encoding="utf-8")
        root_cmake = REPOSITORY_ROOT.joinpath("CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        compute = super_cmake.find(
            "autonomy_compute_enabled_modules(AUTONOMY_ENABLED_MODULES)"
        )
        self.assertGreaterEqual(compute, 0)
        bootstrap = root_cmake.find("autonomy_superproject_bootstrap_modules()")
        find = root_cmake.find("autonomy_find_dependencies()")
        self.assertGreaterEqual(bootstrap, 0)
        self.assertGreater(find, bootstrap)

    def test_build_traverses_only_enabled_modules(self):
        build_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)",
            build_helpers,
        )

    def test_tests_are_discovered_only_in_enabled_modules(self):
        test_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)",
            test_helpers,
        )
        common_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_common.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "add_test(NAME ${NAME} COMMAND $<TARGET_FILE:${NAME}>)",
            common_helpers,
        )

    def test_tests_are_grouped_by_owning_module(self):
        with tempfile.TemporaryDirectory() as directory:
            source = pathlib.Path(directory) / "source"
            build = pathlib.Path(directory) / "build"
            source.mkdir()
            source.joinpath("CMakeLists.txt").write_text(
                textwrap.dedent(
                    f"""
                    cmake_minimum_required(VERSION 3.20)
                    project(autonomy_test_collection_probe LANGUAGES CXX)
                    enable_testing()
                    set(PROJECT_SOURCE_DIR "{REPOSITORY_ROOT.as_posix()}")
                    set(AUTONOMY_ENABLED_MODULES
                        common bridge perception visualization)
                    set(AUTONOMY_BUILD_COMMON_OSQP OFF)
                    set(Ipopt_FOUND OFF)
                    set(BUILD_TEST ON)
                    add_library(autonomy_test_collection_probe INTERFACE)
                    function(autonomy_test NAME ARG_SRC)
                      add_executable("${{NAME}}" "${{ARG_SRC}}")
                      add_test(NAME "${{NAME}}" COMMAND
                        "${{CMAKE_COMMAND}}" -E true)
                      get_property(_registered GLOBAL
                        PROPERTY AUTONOMY_TEST_COLLECTION_REGISTERED)
                      list(APPEND _registered "${{NAME}}")
                      set_property(GLOBAL
                        PROPERTY AUTONOMY_TEST_COLLECTION_REGISTERED
                        "${{_registered}}")
                    endfunction()
                    include("{CMAKE_MODULE_DIR.as_posix()}/autonomy_build.cmake")
                    autonomy_configure_tests()
                    autonomy_add_tests()
                    foreach(_legacy_var IN ITEMS
                        ALL_TESTS TEST_LIBRARY_SRCS TEST_LIBRARY_HDRS)
                      if(DEFINED ${{_legacy_var}})
                        set(_legacy_${{_legacy_var}} true)
                      else()
                        set(_legacy_${{_legacy_var}} false)
                      endif()
                    endforeach()
                    get_property(_registered GLOBAL
                      PROPERTY AUTONOMY_TEST_COLLECTION_REGISTERED)
                    file(WRITE "${{CMAKE_BINARY_DIR}}/collections.txt"
                      "modules=${{AUTONOMY_TEST_MODULES}}\\n"
                      "common_tests=${{AUTONOMY_TESTS_common}}\\n"
                      "common_sources=${{AUTONOMY_TEST_HELPER_SRCS_common}}\\n"
                      "common_headers=${{AUTONOMY_TEST_HELPER_HDRS_common}}\\n"
                      "bridge_tests=${{AUTONOMY_TESTS_bridge}}\\n"
                      "bridge_sources=${{AUTONOMY_TEST_HELPER_SRCS_bridge}}\\n"
                      "bridge_headers=${{AUTONOMY_TEST_HELPER_HDRS_bridge}}\\n"
                      "perception_tests=${{AUTONOMY_TESTS_perception}}\\n"
                      "visualization_tests=${{AUTONOMY_TESTS_visualization}}\\n"
                      "legacy_all_tests=${{_legacy_ALL_TESTS}}\\n"
                      "legacy_helper_srcs=${{_legacy_TEST_LIBRARY_SRCS}}\\n"
                      "legacy_helper_hdrs=${{_legacy_TEST_LIBRARY_HDRS}}\\n"
                      "registered=${{_registered}}\\n")
                    """
                ),
                encoding="utf-8",
            )
            result = subprocess.run(
                ["cmake", "-S", str(source), "-B", str(build)],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            inventory = subprocess.run(
                ["ctest", "--test-dir", str(build), "-N"],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                inventory.returncode, 0, inventory.stdout + inventory.stderr
            )
            collections = dict(
                line.split("=", 1)
                for line in (build / "collections.txt").read_text(
                    encoding="utf-8"
                ).splitlines()
            )

        self.assertEqual(
            collections["modules"], "common;bridge;perception;visualization"
        )
        for module in ("common", "bridge", "perception", "visualization"):
            self.assertIn(f"{module}_tests", collections)
            if collections[f"{module}_tests"]:
                self.assertTrue(
                    all(
                        f"/autonomy/{module}/" in path
                        for path in collections[f"{module}_tests"].split(";")
                    )
                )
        for module in ("common", "bridge"):
            for kind in ("sources", "headers"):
                if collections[f"{module}_{kind}"]:
                    self.assertTrue(
                        all(
                            f"/autonomy/{module}/" in path
                            for path in collections[f"{module}_{kind}"].split(";")
                        )
                    )
        self.assertTrue(collections["common_tests"])
        self.assertTrue(collections["perception_tests"])
        self.assertNotIn("mpc_osqp_test.cpp", collections["common_tests"])
        self.assertNotIn("/optimization/ipopt/", collections["common_tests"])
        self.assertNotIn("/optimization/test/", collections["common_tests"])
        self.assertNotIn("fakedata_test.cpp", collections["visualization_tests"])
        self.assertNotIn("base_component_test.cpp", collections["perception_tests"])
        self.assertNotIn("latest_message_cache.hpp", collections["bridge_headers"])
        self.assertEqual(collections["legacy_all_tests"], "false")
        self.assertEqual(collections["legacy_helper_srcs"], "false")
        self.assertEqual(collections["legacy_helper_hdrs"], "false")
        self.assertIn("autonomy.common.param_handler_test", collections["registered"])
        self.assertIn("autonomy.perception.base.options_test", collections["registered"])
        self.assertIn("autonomy.common.param_handler_test", inventory.stdout)
        self.assertIn("autonomy.perception.base.options_test", inventory.stdout)

    def test_autonomy_test_requires_an_explicit_target(self):
        common_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_common.cmake"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "function(autonomy_test NAME ARG_SRC LINK_TARGET)",
            common_helpers,
        )
        self.assertIn(
            "target_link_libraries(\"${NAME}\" PUBLIC ${LINK_TARGET})",
            common_helpers,
        )
        self.assertIn('"${PROJECT_SOURCE_DIR}"', common_helpers)
        self.assertIn('"${PROJECT_BINARY_DIR}"', common_helpers)
        self.assertNotIn(
            "function(autonomy_test NAME ARG_SRC LINK_TARGET)\n"
            "  add_executable(${NAME} ${ARG_SRC})\n"
            "  _common_compile_stuff()",
            common_helpers,
        )
        self.assertNotIn(
            "target_link_libraries(\"${NAME}\" PUBLIC ${PROJECT_NAME})",
            common_helpers,
        )

        two_argument_calls = []
        for path in REPOSITORY_ROOT.rglob("*"):
            if path.name != "CMakeLists.txt" and path.suffix != ".cmake":
                continue
            relative = path.relative_to(REPOSITORY_ROOT)
            if any(
                part == ".git" or part == "thirdparty" or part.startswith("build")
                for part in relative.parts
            ):
                continue
            content = "\n".join(
                line.split("#", 1)[0]
                for line in path.read_text(encoding="utf-8").splitlines()
            )
            for match in re.finditer(
                r"\bautonomy_test\s*\((.*?)\)", content, re.DOTALL
            ):
                arguments = re.findall(r'"(?:\\.|[^"\\])*"|\S+', match.group(1))
                if len(arguments) < 3:
                    two_argument_calls.append(
                        f"{relative.as_posix()}: {match.group(0)}"
                    )

        self.assertEqual(two_argument_calls, [])

    def test_protobuf_is_discovered_only_in_enabled_modules(self):
        build_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn("function(autonomy_collect_proto_sources)", build_helpers)
        self.assertIn(
            "foreach(_mod IN LISTS AUTONOMY_ENABLED_MODULES)",
            build_helpers,
        )
        # Companion orbisview schemas are not fed into autonomy_proto codegen.
        self.assertNotIn(
            'file(GLOB_RECURSE _orbisview_protos',
            build_helpers,
        )
        self.assertIn(
            "orbisview/*.proto are FE/schema docs",
            build_helpers,
        )

    def test_feature_gates_remain_secondary_to_domain_selection(self):
        super_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_superproject.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "set(AUTONOMY_MODULE_CONDITION_bridge BUILD_GRPC)",
            super_cmake,
        )
        self.assertIn(
            "set(AUTONOMY_MODULE_CONDITION_visualization foxglove-sdk_FOUND)",
            super_cmake,
        )

    def test_root_reports_explicitly_disabled_domains(self):
        super_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_superproject.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '"autonomy: module \'${_module}\' disabled by "',
            super_cmake,
        )
        self.assertIn(
            '"AUTONOMY_BUILD_${_module_upper}=OFF"',
            super_cmake,
        )

    def test_superproject_sets_lazy_load_flag(self):
        root_cmake = REPOSITORY_ROOT.joinpath("CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        module_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_module.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn("set(AUTONOMY_SUPERPROJECT ON)", root_cmake)
        self.assertIn("if(AUTONOMY_SUPERPROJECT)", module_cmake)

    def test_common_osqp_is_folded_into_autonomy_common(self):
        common_cmake = REPOSITORY_ROOT.joinpath(
            "autonomy/common/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertIn("if(AUTONOMY_BUILD_COMMON_OSQP)", common_cmake)
        self.assertIn("list(APPEND _COMMON_FEATURES osqp)", common_cmake)
        self.assertIn("mpc_osqp", common_cmake)
        self.assertNotIn("add_library(autonomy_common_osqp SHARED", common_cmake)
        self.assertNotIn("autonomy::common_osqp", common_cmake)

    def test_common_ipopt_sources_require_discovered_ipopt(self):
        common_cmake = REPOSITORY_ROOT.joinpath(
            "autonomy/common/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertIn("if(NOT Ipopt_FOUND)", common_cmake)
        self.assertIn(
            'list(FILTER _COMMON_SRCS EXCLUDE REGEX "/optimization/ipopt/")',
            common_cmake,
        )
        tests_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn("if(NOT Ipopt_FOUND)", tests_cmake)
        self.assertIn('list(FILTER _tests EXCLUDE REGEX "/optimization/(ipopt|test)/")', tests_cmake)

    def test_osqp_test_is_filtered_when_osqp_is_disabled(self):
        tests_cmake = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertIn("if(NOT AUTONOMY_BUILD_COMMON_OSQP)", tests_cmake)
        self.assertIn(
            '"${_root}/common/math/mpc_osqp_test.cpp"',
            tests_cmake,
        )

    def test_test_support_does_not_relink_gmock(self):
        build_helpers = REPOSITORY_ROOT.joinpath(
            "cmake/autonomy_build.cmake"
        ).read_text(encoding="utf-8")
        self.assertNotIn(
            "target_link_libraries(${TEST_LIB} PUBLIC ${GMOCK_LIBRARY}",
            build_helpers,
        )
        self.assertNotIn(
            "target_link_libraries(${TEST_LIB} INTERFACE ${GMOCK_LIBRARY}",
            build_helpers,
        )


if __name__ == "__main__":
    unittest.main()
