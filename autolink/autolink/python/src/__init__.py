#!/usr/bin/env python3

#***************************************************************************
# Copyright 2025 The Openbot Authors (duyongquan)
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
#***************************************************************************

import sys

if sys.version_info[0] < 3:
    sys.stderr.write('''
        You are running Python2 while importing Python3 Autolink wrapper!
        Please change to "import autolink_py.xyz" accordingly.\n''')
    sys.exit(1)

# Unified package exports:
# - keep module-level imports available (autolink/parameter/record/time/timer)
# - expose commonly used classes/functions at package top-level
from . import autolink  # noqa: F401
from . import parameter  # noqa: F401
from . import record  # noqa: F401
from . import time as autolink_time  # noqa: F401
from . import timer  # noqa: F401

from .autolink import (  # noqa: F401
    Node,
    NodeUtils,
    ChannelUtils,
    ServiceUtils,
    Client,
    Reader,
    Writer,
    init,
    ok,
    shutdown,
    is_shutdown,
    waitforshutdown,
)
from .parameter import Parameter, ParameterClient, ParameterServer  # noqa: F401
from .record import RecordReader, RecordWriter, PyBagMessage  # noqa: F401
from .time import Duration, Time, Rate  # noqa: F401
from .timer import Timer  # noqa: F401

__all__ = [
    "autolink",
    "parameter",
    "record",
    "autolink_time",
    "timer",
    "Node",
    "NodeUtils",
    "ChannelUtils",
    "ServiceUtils",
    "Client",
    "Reader",
    "Writer",
    "init",
    "ok",
    "shutdown",
    "is_shutdown",
    "waitforshutdown",
    "Parameter",
    "ParameterClient",
    "ParameterServer",
    "RecordReader",
    "RecordWriter",
    "PyBagMessage",
    "Duration",
    "Time",
    "Rate",
    "Timer",
]