# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Module entry: ``python -m autosim``."""

import sys


def run() -> None:
    """Start the sensor–actuator bridge with the default config."""
    # Keep --list usable without importing the heavier runtime/autolink stack.
    if "--list" in sys.argv[1:]:
        from autosim.runtime.presets import format_catalog

        print(format_catalog())
        return

    from autosim.runtime import Runner

    Runner.main()


if __name__ == "__main__":
    run()
