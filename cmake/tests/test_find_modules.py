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

    def test_eigen_uses_config_package(self):
        result = self.configure(
            """
            find_package(Eigen3 REQUIRED CONFIG)
            if(NOT TARGET Eigen3::Eigen)
              message(FATAL_ERROR "Eigen3::Eigen missing")
            endif()
            if(NOT Eigen3_FOUND OR NOT EIGEN3_FOUND OR NOT EIGEN3_INCLUDE_DIRS)
              message(FATAL_ERROR "Eigen compatibility variables missing")
            endif()
            """,
            {
                "share/eigen3/cmake/Eigen3Config.cmake": textwrap.dedent(
                    """
                    set(Eigen3_FOUND TRUE)
                    set(EIGEN3_FOUND TRUE)
                    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../include/eigen3")
                    set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
                    add_library(Eigen3::Eigen INTERFACE IMPORTED)
                    set_target_properties(Eigen3::Eigen PROPERTIES
                      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
                    """
                ),
                "include/eigen3/Eigen/Core": "// fixture\n",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_glog_uses_config_package(self):
        result = self.configure(
            """
            find_package(glog REQUIRED CONFIG)
            if(NOT TARGET glog::glog)
              message(FATAL_ERROR "glog::glog missing")
            endif()
            """,
            {
                "lib/cmake/glog/glog-config.cmake": textwrap.dedent(
                    """
                    add_library(glog::glog INTERFACE IMPORTED)
                    set_target_properties(glog::glog PROPERTIES
                      INTERFACE_INCLUDE_DIRECTORIES
                        "${CMAKE_CURRENT_LIST_DIR}/../../../include")
                    """
                ),
                "include/glog/logging.h": "// fixture\n",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_ceres_uses_config_package(self):
        result = self.configure(
            """
            find_package(Ceres REQUIRED CONFIG)
            if(NOT TARGET Ceres::ceres)
              message(FATAL_ERROR "Ceres::ceres missing")
            endif()
            """,
            {
                "lib/cmake/Ceres/CeresConfig.cmake": textwrap.dedent(
                    """
                    set(Ceres_FOUND TRUE)
                    set(CERES_FOUND TRUE)
                    add_library(Ceres::ceres INTERFACE IMPORTED)
                    set_target_properties(Ceres::ceres PROPERTIES
                      INTERFACE_INCLUDE_DIRECTORIES
                        "${CMAKE_CURRENT_LIST_DIR}/../../../include")
                    """
                ),
                "include/ceres/ceres.h": "// fixture\n",
            },
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_hand_written_eigen_ceres_glog_finds_removed(self):
        modules = REPOSITORY_ROOT / "cmake" / "modules"
        self.assertFalse((modules / "FindEigen3.cmake").exists())
        self.assertFalse((modules / "FindCeres.cmake").exists())
        self.assertFalse((modules / "FindGlog.cmake").exists())


if __name__ == "__main__":
    unittest.main()
