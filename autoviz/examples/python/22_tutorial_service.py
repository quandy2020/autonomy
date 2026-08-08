#!/usr/bin/env python3
"""Host /fake/echo String service for Autoviz Service Call panel.

  Service: /fake/echo  std_msgs/String → echo request

  /usr/bin/python3 examples/python/22_tutorial_service.py
  ./build/autonomy/bin/autoviz
"""

from __future__ import annotations

from _reexec import early_reexec_if_needed

early_reexec_if_needed()

import time

from _bootstrap import prepare_example_environment

prepare_example_environment()

import autolink
from automsgs.msgs.std_msgs import string_pb2


def echo(req):
    print(f'echo ← {req.data!r}')
    return string_pb2.String(data=f'echo:{req.data}')


def main():
    autolink.init('service')
    n = autolink.Node('/autoviz/service')
    n.create_service('/fake/echo', echo, req_type=string_pb2.String)
    print('Service /fake/echo ready — use Autoviz Service Call panel')
    try:
        while not autolink.is_shutdown():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        autolink.shutdown()


if __name__ == '__main__':
    main()
