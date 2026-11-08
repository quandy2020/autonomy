"""Regression tests for the standalone navigation processes."""
import selectors
import subprocess
import sys
import time

def check_supervisor(argv):
    with subprocess.Popen(argv, stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, bufsize=0) as supervisor:
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(supervisor.stdout, selectors.EVENT_READ)
                if not selector.select(3) or supervisor.stdout.readline() != b"1\n":
                    sys.exit("supervisor did not start in stop mode")
                # Match the system C++ steady_clock epoch, not Python's process origin.
                clock = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
                         else time.CLOCK_MONOTONIC)
                now = time.clock_gettime(clock)
                modules = ("safety", "behavior", "perception", "policy", "reflex", "memory")
                for module in modules:
                    supervisor.stdin.write(f"{now} {module} 0 {now} 1\n".encode())
                modes = []
                deadline = time.monotonic() + 3
                while selector.select(max(0, deadline - time.monotonic())):
                    line = supervisor.stdout.readline()
                    if not line:
                        break
                    modes.append(int(line))
                    if 0 in modes and modes[-1] == 1:
                        break
                    if time.monotonic() >= deadline:
                        break
            if 0 not in modes or modes[-1] != 1:
                sys.exit(f"expected normal then stop with stdin still open, got {modes}")
        finally:
            supervisor.terminate()


def check_safety(argv):
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
    with subprocess.Popen(argv, stdin=subprocess.PIPE,
                          stdout=subprocess.PIPE, bufsize=0) as safety:
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(safety.stdout, selectors.EVENT_READ)

                def wait_for(predicate, timeout):
                    deadline = time.monotonic() + timeout
                    while selector.select(max(0, deadline - time.monotonic())):
                        fields = safety.stdout.readline().split()
                        if not fields or time.monotonic() >= deadline:
                            break
                        if predicate(fields):
                            return
                    raise AssertionError("safety did not emit the expected command")

                is_stop = lambda fields: fields[:3] == [b"0", b"0", b"1"]
                wait_for(is_stop, 3)
                now = time.clock_gettime(clock)
                safety.stdin.write(f"v2 1 {now} {now} {now} 0.2 0 0 0 3 0\n".encode())
                wait_for(lambda fields: fields[2] == b"0" and float(fields[0]) > 0, 1)
                now = time.clock_gettime(clock)
                safety.stdin.write(f"v2 2 {now} {now} {now - 2} 0.2 0 0 0 3 0\n".encode())
                # The previous packet would still be within the 200 ms lease.
                wait_for(is_stop, 0.12)
                now = time.clock_gettime(clock)
                safety.stdin.write(f"v2 3 {now} {now} {now} 0.2 0 0 0 3 0\n".encode())
                wait_for(lambda fields: fields[2] == b"0" and float(fields[0]) > 0, 1)
                now = time.clock_gettime(clock)
                safety.stdin.write(f"v2 4 {now} {now - 2} {now} 0.2 0 0 0 3 0\n".encode())
                wait_for(is_stop, 0.12)
        finally:
            safety.terminate()


if __name__ == "__main__":
    {"supervisor": check_supervisor, "safety": check_safety}[sys.argv[1]](sys.argv[2:])
