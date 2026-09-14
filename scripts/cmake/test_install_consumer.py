#!/usr/bin/env python3

import pathlib
import subprocess
import tempfile
import textwrap
import unittest


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD_DIR = REPOSITORY_ROOT / "build" / "autonomy-minimal"


class InstallConsumerTest(unittest.TestCase):
    def run_command(self, command, cwd=None):
        result = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.returncode,
            0,
            result.stdout + result.stderr,
        )

    def test_minimal_install_is_consumable(self):
        if not (BUILD_DIR / "CMakeCache.txt").exists():
            self.skipTest("configure the autonomy-minimal preset first")

        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            prefix = root / "install"
            source = root / "consumer"
            build = root / "build"
            source.mkdir()

            source.joinpath("CMakeLists.txt").write_text(
                textwrap.dedent(
                    """
                    cmake_minimum_required(VERSION 3.20)
                    project(autonomy_consumer LANGUAGES CXX)

                    find_package(autonomy CONFIG REQUIRED)
                    add_executable(autonomy_consumer main.cpp)
                    target_link_libraries(
                      autonomy_consumer PRIVATE autonomy::autonomy)
                    """
                ),
                encoding="utf-8",
            )
            source.joinpath("main.cpp").write_text(
                textwrap.dedent(
                    """
                    #include "autonomy/common/time.hpp"

                    int main() {
                      const auto time = autonomy::common::FromUniversal(0);
                      (void)time;
                      return 0;
                    }
                    """
                ),
                encoding="utf-8",
            )

            self.run_command(
                ["cmake", "--install", str(BUILD_DIR), "--prefix", str(prefix)]
            )
            for disabled_module in ("map", "localization", "perception", "task"):
                self.assertFalse(
                    (prefix / "include" / "autonomy" / disabled_module).exists(),
                    disabled_module,
                )
                self.assertFalse(
                    (prefix / "share" / "autonomy" / disabled_module).exists(),
                    disabled_module,
                )
            self.run_command(
                [
                    "cmake",
                    "-S",
                    str(source),
                    "-B",
                    str(build),
                    "-G",
                    "Ninja",
                    f"-DCMAKE_PREFIX_PATH={prefix}",
                ]
            )
            self.run_command(["cmake", "--build", str(build)])
            self.run_command([str(build / "autonomy_consumer")])


if __name__ == "__main__":
    unittest.main()
