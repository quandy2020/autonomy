#!/usr/bin/env python3
"""Write an allowlisted release audit; never dump inventory variables or secrets."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment', choices=['staging', 'production'], required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--deployment-revision', help='Commit of the deployment tooling and inventory')
    parser.add_argument('--limit', default='autonomy')
    parser.add_argument('--artifact-sha256', required=True)
    parser.add_argument('--cross-profile', default='')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if not re.fullmatch('[0-9a-f]{40}', args.revision) or not re.fullmatch('[0-9a-f]{64}', args.artifact_sha256):
        parser.error('release revision and artifact checksum must be exact hashes')
    if args.cross_profile and not re.fullmatch(r'[a-z0-9]+(?:-[a-z0-9]+)+', args.cross_profile):
        parser.error('cross profile must be a canonical Profile ID')
    root = Path(__file__).resolve().parents[2]
    ansible_root = root / 'ansible'
    deployment_revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if deployment_revision != (args.deployment_revision or args.revision):
        parser.error('checkout revision does not match the requested deployment tooling')
    inventory_tree = subprocess.check_output(
        ['git', '-C', str(root), 'rev-parse', f'HEAD:ansible/inventory/{args.environment}'], text=True).strip()
    hosts = subprocess.check_output(
        ['ansible-playbook', '-i', f'inventory/{args.environment}/hosts.yml',
         f'playbooks/{args.environment}.yml', '--list-hosts', '--limit', args.limit], cwd=ansible_root, text=True)
    if not any(int(count) > 0 for count in re.findall(r'hosts \((\d+)\):', hosts)):
        parser.error('host selector matched no deployment targets')
    audit = {
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'revision': args.revision,
        'deployment_revision': deployment_revision,
        'inventory_tree': inventory_tree,
        'environment': args.environment,
        'artifact_sha256': args.artifact_sha256,
        'cross_profile': args.cross_profile,
        'lock_sha256': hashlib.sha256((ansible_root / 'vars/lock.yml').read_bytes()).hexdigest(),
        'actor': os.environ.get('GITHUB_ACTOR', 'local'),
        'artifact_source_run_id': os.environ.get('AUTONOMY_ARTIFACT_RUN_ID', os.environ.get('GITHUB_RUN_ID', '')),
        'run_url': '/'.join([os.environ.get('GITHUB_SERVER_URL', 'https://github.com'),
                             os.environ.get('GITHUB_REPOSITORY', ''), 'actions/runs',
                             os.environ.get('GITHUB_RUN_ID', '')]),
        'host_listing': hosts,
        'host_limit': args.limit,
        'extra_variables': {'autonomy_play_mode': 'full', 'autonomy_deploy_mode': 'artifact',
                            'autonomy_expected_git_sha': args.revision,
                            'autonomy_artifact_sha256': args.artifact_sha256},
    }
    args.output.write_text(json.dumps(audit, indent=2) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
