#!/usr/bin/env python3

import importlib
import os
import sys


def _find_wrapper_lib_path():
    autolink_home = os.environ.get("AUTOLINK_DISTRIBUTION_HOME", "/opt/apollo/neo")
    if autolink_home:
        test_path = os.path.join(autolink_home, "lib", "autolink", "python", "internal")
        if os.path.exists(test_path):
            return test_path

    for prefix in ("/usr/local", "/usr"):
        test_path = os.path.join(prefix, "lib", "autolink", "python", "internal")
        if os.path.exists(test_path):
            return test_path

    for path in sys.path:
        if not os.path.isdir(path):
            continue
        if "site-packages" in path or "dist-packages" in path:
            prefix = os.path.dirname(os.path.dirname(path))
            test_path = os.path.join(prefix, "lib", "autolink", "python", "internal")
            if os.path.exists(test_path):
                return test_path
    return None


def load_wrapper_module(module_name):
    wrapper_lib_path = _find_wrapper_lib_path()
    if wrapper_lib_path and wrapper_lib_path not in sys.path:
        sys.path.append(wrapper_lib_path)
    return importlib.import_module(module_name)
