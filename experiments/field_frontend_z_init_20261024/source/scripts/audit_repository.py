#!/usr/bin/env python3
"""Check the exact Git upload/index for generated files, secrets and machine paths."""
import argparse
from pathlib import Path
import re
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--staged', action='store_true', help='inspect index blobs rather than worktree files')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    entries = subprocess.check_output(['git', 'ls-files', '--stage', '-z'], cwd=root).split(b'\0')
    bad = []
    count = total = 0
    paths = []
    for entry in entries:
        if not entry:
            continue
        meta, raw_path = entry.split(b'\t', 1)
        mode = meta.split()[0]
        path = raw_path.decode()
        if mode == b'160000':
            continue
        paths.append(path)
        if args.staged:
            content = subprocess.check_output(['git', 'show', ':' + path], cwd=root)
        else:
            content = (root / path).read_bytes()
        count += 1
        total += len(content)
        top = path.split('/')[0]
        if (top in {'log', 'logs', 'artifacts', 'backups', 'output', '.deps', '.superpowers'}
                or top.startswith(('build', 'install')) and top not in {'build.sh'}
                or '__pycache__' in path or path.endswith(('.orig', '.bak', '.log', '.pyc', '.stp', '.step'))):
            bad.append((path, 'generated/backup/data file'))
        if len(content) > 20 * 1024 * 1024:
            bad.append((path, 'file exceeds 20 MiB source/resource allowance'))
        if path.lower().endswith('.pcd') and not path.startswith('src/a2w_fastlio_common/test/data/real_fine_failure/'):
            bad.append((path, 'raw point cloud outside the documented small regression fixture'))
        if path.lower().endswith(('.stl', '.pcd')):
            continue
        if b'\0' in content:
            bad.append((path, 'unexpected binary file'))
            continue
        text = content.decode('utf-8', errors='replace')
        if re.search(r'/home/[A-Za-z0-9_-]+(?:/|\b)', text):
            bad.append((path, 'machine-specific home path'))
        if re.search(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|github_pat_[A-Za-z0-9_]{30,}|gh[pousr]_[A-Za-z0-9]{30,}|AKIA[A-Z0-9]{16}', text):
            bad.append((path, 'possible embedded credential (value not printed)'))
        if re.search(r'sshpass\s+-p\s+[\'\"]?[A-Za-z0-9]+(?:[\'\"]?\s|$)', text):
            bad.append((path, 'literal SSH password'))
    if len(paths) != len(set(paths)):
        bad.append(('<index>', 'unmerged/duplicate index entries'))
    for path, reason in bad:
        print(f'FAIL {path}: {reason}', file=sys.stderr)
    print(f'Checked {count} source/resource files, {total / 1024**2:.2f} MiB; failures={len(bad)}')
    return int(bool(bad))


if __name__ == '__main__':
    raise SystemExit(main())
