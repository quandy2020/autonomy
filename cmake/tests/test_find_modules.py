#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import textwrap
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
CMAKE_MODULE_DIR = REPOSITORY_ROOT / "cmake" / "modules"


class FindModulesTest(unittest.TestCase):
    def configure(self, body, files):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            prefix = root / "prefix"
            source = root / "source"
            build = root / "build"
            source.mkdir()
            for relative_path, content in files.items():
                path = prefix / relative_path
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            source.joinpath("CMakeLists.txt").write_text(
                textwrap.dedent(
                    f"""
                    cmake_minimum_required(VERSION 3.20)
                    project(find_module_contract LANGUAGES NONE)
                    list(PREPEND CMAKE_MODULE_PATH "{CMAKE_MODULE_DIR.as_posix()}")
                    list(PREPEND CMAKE_PREFIX_PATH "{prefix.as_posix()}")
                    {body}
                    """
                ),
                encoding="utf-8",
            )
            return subprocess.run(
                ["cmake", "-Werror=dev", "-S", str(source), "-B", str(build)],
                check=False,
                capture_output=True,
                text=True,
            )

    def test_eigen_config_mode_sets_imported_target_and_legacy_variables(self):
        result = self.configure(
            """
            find_package(Eigen3 REQUIRED)
            if(NOT TARGET Eigen3::Eigen)
              message(FATAL_ERROR "Eigen3::Eigen missing")
            endif()
            if(NOT Eigen3_FOUND OR NOT EIGEN3_FOUND OR NOT EIGEN3_INCLUDE_DIRS)
              message(FATAL_ERROR "Eigen compatibility variables missing")
            endif()
            """,
            {
                "lib/cmake/eigen3/Eigen3Config.cmake": textwrap.dedent(
                    """
                    set(Eigen3_FOUND TRUE)
                    set(Eigen3_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include/eigen3")
                    add_library(Eigen3::Eigen INTERFACE IMPORTED)
                    set_target_properties(Eigen3::Eigen PROPERTIES
                      INTERFACE_INCLUDE_DIRECTORIES "${Eigen3_INCLUDE_DIR}")
                    """
                ),
                "include/eigen3/Eigen/Core": "// fixture\n",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_glog_package_name_matches_find_module_case(self):
        result = self.configure(
            """
            find_package(Glog REQUIRED)
            if(NOT TARGET glog::glog)
              message(FATAL_ERROR "glog::glog missing")
            endif()
            if(NOT Glog_FOUND OR NOT GLOG_FOUND OR NOT GLOG_INCLUDE_DIRS OR NOT GLOG_LIBRARIES)
              message(FATAL_ERROR "Glog compatibility variables missing")
            endif()
            """,
            {
                "include/glog/logging.h": "// fixture\n",
                "lib/libglog.dylib": "",
                "lib/libglog.so": "",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_ceres_sets_imported_target_and_legacy_variables(self):
        result = self.configure(
            """
            find_package(Ceres REQUIRED)
            if(NOT TARGET Ceres::ceres)
              message(FATAL_ERROR "Ceres::ceres missing")
            endif()
            if(NOT Ceres_FOUND OR NOT CERES_FOUND OR NOT CERES_INCLUDE_DIRS OR NOT CERES_LIBRARIES)
              message(FATAL_ERROR "Ceres compatibility variables missing")
            endif()
            """,
            {
                "include/ceres/ceres.h": "// fixture\n",
                "lib/libceres.dylib": "",
                "lib/libceres.so": "",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
