#!/usr/bin/env python3
"""Example external release-event receiver. Never sends body or finger commands."""
import argparse,json,os,socket
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--socket',default='/tmp/g1_throw_release.sock');a=p.parse_args()
    path=Path(a.socket)
    if path.exists():raise SystemExit(f'{path} already exists; use the actual hand receiver or remove its stale socket after checking the owner.')
    receiver=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    receiver.bind(str(path));os.chmod(path,0o600)
    seen=set()
    print(f'Listening on {path}; prints events only, does not control the Inspire hand.',flush=True)
    try:
        while True:
            message=json.loads(receiver.recv(8192))
            key=(message.get('pid'),message.get('trial_id'))
            if message.get('event')!='release' or key in seen:continue
            seen.add(key);print(json.dumps(message,ensure_ascii=False),flush=True)
            # Integrators call their separately deployed right-hand OPEN API here.
            # Do not assume socket delivery proves the apple has physically left.
    except KeyboardInterrupt:pass
    finally:receiver.close();path.unlink(missing_ok=True)


if __name__=='__main__':main()
