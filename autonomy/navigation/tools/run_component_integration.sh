#!/usr/bin/env bash
# Keep real Autolink transport in a private IPC, mount and network namespace.
set -euo pipefail
exec unshare --ipc --mount --net --fork bash -c '
  set -euo pipefail
  mount --make-rprivate /
  mount -t tmpfs -o size=64m tmpfs /dev/shm
  mount -t tmpfs -o size=64m tmpfs /tmp
  ip link set lo up multicast on
  ip route add 224.0.0.0/4 dev lo
  export AUTOLINK_IP=127.0.0.1
  exec "$@"
' navigation-component-test "$@"
