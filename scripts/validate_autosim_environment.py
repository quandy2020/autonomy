#!/usr/bin/env python3
"""Record real AutoSim sensor publications before an exploration acceptance run.

Run with the same Python, Autolink bindings and generated automsgs as AutoSim.
The simulator backend is constructed by AutoSim's actual registry. Fake backends
and shared command channels are rejected. This program sends no motion command
and does not report exploration completion or substitute ground truth for an
algorithm map. All recorded payloads are existing automsgs protobuf messages.
"""

import argparse
import hashlib
import json
from pathlib import Path
import threading
import time


def main():
    """Run a bounded, isolated simulator and retain immutable transport evidence."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=250)
    args = parser.parse_args()
    if not 50 <= args.steps <= 10000:
        parser.error('steps must be in [50,10000]')
    args.output.mkdir(parents=True, exist_ok=False)

    from autosim.runtime.settings import Config
    from autosim.runtime.runner import Runner
    from autosim.runtime.instance_lock import InstanceLock

    config = Config.load(args.config)
    backend = config.data['habitat']['backend']
    if backend not in ('habitat', 'isaac'):
        parser.error('acceptance requires the real Habitat or Isaac backend')
    channels = config.channel_map()
    required = ('rgb', 'depth', 'camera_info', 'map_grid', 'gt_pose', 'odom', 'clock')
    if any(key not in channels for key in required):
        parser.error('configuration lacks required sensor/truth recording channels')
    if any(not value.startswith('/validation/exploration/') for value in channels.values()):
        parser.error('every channel, including cmd_vel, must be isolated under /validation/exploration/')

    start = time.monotonic()
    lock = threading.Lock()
    counts = {key: 0 for key in required}
    first = {}
    last_arrival = {}
    reader_owners = []
    report = {
        'passed': False, 'backend': backend,
        'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
        'channels': channels, 'maximum_steps': args.steps,
        'exploration_task_started': False, 'coverage_comparison_performed': False,
        'scope': 'Real simulator startup and native sensor transport only; no exploration acceptance.',
    }
    try:
        with InstanceLock.from_environment(args.config):
            runner = Runner(config, max_steps=args.steps)
            types = runner.message_types()
            for key in required:
                def receive(message, name=key):
                    if message is None:
                        return
                    with lock:
                        counts[name] += 1
                        last_arrival[name] = time.monotonic() - start
                        if name not in first:
                            first[name] = message.SerializeToString()
                reader_owners.append(runner.node.create_reader(channels[key], receive, types[key]))
            runner.run()
        with lock:
            report['message_counts'] = dict(counts)
            report['last_arrival_seconds'] = dict(last_arrival)
            payloads = dict(first)
        report['payload_sha256'] = {}
        for name, data in payloads.items():
            (args.output / (name + '.pb')).write_bytes(data)
            report['payload_sha256'][name] = hashlib.sha256(data).hexdigest()
        missing = [key for key, count in counts.items() if count < 2]
        report['missing_or_nonrepeating_streams'] = missing
        report['passed'] = not missing
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        report['elapsed_seconds'] = time.monotonic() - start
        with (args.output / 'report.json').open('x') as output:
            json.dump(report, output, indent=2)
            output.write('\n')
    print(json.dumps(report), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
