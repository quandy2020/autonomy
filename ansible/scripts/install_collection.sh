#!/usr/bin/env bash
# Install the local collection once per source and installer revision.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ANSIBLE_CONFIG="${ROOT}/ansible.cfg"

exec python3 - "${ROOT}" <<'PY'
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

root = Path(sys.argv[1])
store = root / '.ansible'
store.mkdir(exist_ok=True)
collection = store / 'collections/ansible_collections/openbot/autonomy_ops'
stamp = store / '.autonomy-collection.sha256'
with (store / '.autonomy-collection.lock').open('a') as guard:
    fcntl.flock(guard, fcntl.LOCK_EX)
    digest = hashlib.sha256(subprocess.check_output(['ansible-galaxy', '--version']))
    # Include the versioned collection roots. Inventories, generated work and credentials are separate.
    paths = [root / 'galaxy.yml', root / 'README.md', root / 'CHANGELOG.md', root / 'DESIGN.md']
    for subtree in ('roles', 'playbooks', 'meta', 'plugins', 'scripts', 'docs', 'vars'):
        paths.extend(path for path in (root / subtree).rglob('*') if path.is_file())
    for path in sorted(paths):
        if '__pycache__' in path.parts or path.suffix == '.pyc':
            continue
        digest.update(os.fsencode(path.relative_to(root)) + b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    key = digest.hexdigest()

    try:
        entries = json.loads((collection / 'FILES.json').read_text())['files']
        valid = (collection / 'MANIFEST.json').is_file() and all(
            (collection / entry['name']).exists() for entry in entries
        )
        if valid and stamp.read_text().strip() == key:
            print('[collection] local cache hit')
            sys.exit(0)
    except (OSError, ValueError, KeyError, TypeError):
        pass

    stamp.unlink(missing_ok=True)
    begin = time.monotonic()
    subprocess.run(['ansible-galaxy', 'collection', 'install', str(root), '--force', '--no-deps',
                    '--collections-path', str(store / 'collections')], check=True)
    temporary = stamp.with_suffix('.tmp')
    temporary.write_text(key + '\n')
    os.replace(temporary, stamp)
    print(f'[collection] installed in {time.monotonic() - begin:.2f}s')
PY
