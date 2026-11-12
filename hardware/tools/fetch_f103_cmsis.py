#!/usr/bin/env python3
"""Fetch only the pinned official CMSIS files; do not modify board or system."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    records = json.loads((ROOT/'configs/f103_cmsis_sources.json').read_text())
    base = ROOT/'vendor/stm32f1_cmsis'
    for item in records:
        path = base/item['path']
        if path.exists():
            data = path.read_bytes()
        else:
            data = urllib.request.urlopen(item['url'], timeout=30).read()
        if hashlib.sha256(data).hexdigest() != item['sha256']:
            raise RuntimeError(f'Source hash mismatch: {path}; existing files will not be replaced')
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    (base/'sources.json').write_text(json.dumps(records, indent=2)+'\n')
    print('Pinned CMSIS files verified:', len(records))


if __name__ == '__main__':
    main()
