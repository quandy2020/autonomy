#!/usr/bin/env python3
"""Check a running Autonomy Bridge with a read-only heartbeat RPC."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bridge', required=True, type=Path, help='Absolute path to autonomy.bridge')
    parser.add_argument('--target', required=True, help='Bridge address in host:port form')
    parser.add_argument('--timeout', type=int, default=3, help='Maximum seconds for the RPC')
    args = parser.parse_args()
    if not args.bridge.is_absolute() or args.timeout <= 0:
        parser.error('--bridge must be absolute and --timeout must be positive')

    try:
        result = subprocess.run([
            str(args.bridge), 'call', 'SystemService/Heartbeat', '-d', '{"sequence":42}',
            '--target', args.target, '--timeout', str(args.timeout),
        ], capture_output=True, text=True, timeout=args.timeout + 1, check=True)
        heartbeat = json.loads(result.stdout)
        status = heartbeat['status']
        if status.get('code', 0) not in (0, 'STATUS_OK', 'OK'):
            raise ValueError('heartbeat returned a non-OK application status')
        if int(heartbeat['sequence']) != 42 or int(heartbeat['robot_time_ns']) <= 0:
            raise ValueError('heartbeat returned an unexpected sequence or timestamp')
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError, AttributeError) as error:
        parser.exit(1, f'Bridge probe failed: {error}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
