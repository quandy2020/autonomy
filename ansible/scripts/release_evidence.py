#!/usr/bin/env python3
"""Record and verify the exact artifact accepted by a completed staging rollout."""
import argparse
import json
from pathlib import Path
import re


def record(audit):
    if audit.get('environment') != 'staging':
        raise ValueError('only a staging rollout can produce acceptance evidence')
    revision, checksum = audit.get('revision', ''), audit.get('artifact_sha256', '')
    if not (re.fullmatch('[0-9a-f]{40}', revision) and re.fullmatch('[0-9a-f]{64}', checksum)):
        raise ValueError('staging audit must contain a commit SHA and artifact SHA-256')
    if not re.search(r'hosts \([1-9][0-9]*\):', audit.get('host_listing', '')):
        raise ValueError('staging must have selected at least one target')
    profile = audit.get('cross_profile', '')
    if profile and not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)+', profile):
        raise ValueError('invalid cross profile in staging audit')
    return {
        'schema': 1, 'result': 'verified', 'environment': 'staging',
        'revision': revision, 'artifact_sha256': checksum, 'cross_profile': profile,
        'inventory_tree': audit['inventory_tree'], 'host_limit': audit['host_limit'],
    }


def verify(evidence, revision, checksum, profile):
    if (evidence.get('schema'), evidence.get('result'), evidence.get('environment')) != (1, 'verified', 'staging'):
        raise ValueError('staging evidence must come from a completed and verified deployment')
    if (evidence.get('revision'), evidence.get('artifact_sha256'), evidence.get('cross_profile')) != (
            revision, checksum, profile):
        raise ValueError('staging evidence does not match the requested commit, artifact or profile')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='mode', required=True)
    accept = commands.add_parser('record')
    accept.add_argument('audit', type=Path)
    accept.add_argument('output', type=Path)
    promote = commands.add_parser('verify')
    promote.add_argument('evidence', type=Path)
    promote.add_argument('--revision', required=True)
    promote.add_argument('--sha256', required=True)
    promote.add_argument('--cross-profile', default='')
    args = parser.parse_args()
    try:
        if args.mode == 'record':
            args.output.write_text(json.dumps(record(json.loads(args.audit.read_text())), indent=2) + '\n')
        else:
            verify(json.loads(args.evidence.read_text()), args.revision, args.sha256, args.cross_profile)
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        parser.exit(1, f'Release evidence: {error}\n')


if __name__ == '__main__':
    main()
