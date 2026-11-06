#!/usr/bin/env python3
"""Integrity check only; no SDK import or robot connection."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'manifest.json').read_text())
for name,expected in manifest['critical_files_sha256'].items():
    p=root/name
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Integrity mismatch: '+name)
print('Policy/config/source checksums OK; no robot commands sent.')
