#!/usr/bin/env python3
"""Install hash-pinned Ubuntu packages inside .toolchain only; no sudo or USB IO."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import urllib.error
import urllib.request

ROOT=Path(__file__).resolve().parents[1]


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache-dir',type=Path,help='Optional existing package cache, searched recursively')
    p.add_argument('--verify-only',action='store_true',help='Only verify local package files, do not fetch/extract')
    args=p.parse_args()
    if platform.machine() not in ('x86_64','amd64'):p.error('Pinned tools target Ubuntu 22.04 x86_64; not Raspberry Pi')
    target=ROOT/'.toolchain';cache=target/'packages';cache.mkdir(parents=True,exist_ok=True)
    records=json.loads((ROOT/'configs/toolchain_packages.json').read_text())['packages']
    for record in records:
        package=cache/record['filename']
        if not package.exists() and args.cache_dir:
            candidates=list(args.cache_dir.rglob(record['filename']))
            for candidate in candidates:
                if candidate.resolve()!=package.resolve() and digest(candidate)==record['sha256']:
                    shutil.copyfile(candidate,package);break
        if not package.exists():
            if args.verify_only:raise RuntimeError('Package missing: '+str(package))
            failures=[]
            for url in record['urls']:
                temporary=package.with_suffix(package.suffix+'.part')
                try:
                    with urllib.request.urlopen(url,timeout=60) as remote, temporary.open('wb') as out:
                        shutil.copyfileobj(remote,out)
                    if digest(temporary)!=record['sha256']:raise RuntimeError('Downloaded SHA-256 mismatch')
                    temporary.replace(package);break
                except (OSError,RuntimeError) as error:
                    temporary.unlink(missing_ok=True);failures.append(str(error))
            else:raise RuntimeError('Cannot fetch '+record['package']+': '+'; '.join(failures))
        if digest(package)!=record['sha256']:raise RuntimeError('Cached package SHA-256 mismatch: '+str(package))
        print(('Verified ' if args.verify_only else 'Extracting ')+record['package']+' '+record['version'],flush=True)
        if not args.verify_only:subprocess.run(['dpkg-deb','-x',str(package),str(target)],check=True)
    if not args.verify_only:
        base=target/'usr/lib/arm-none-eabi'
        for name,destination in (('include','../../include/newlib'),('lib','newlib')):
            link=base/name
            if not link.exists() and not link.is_symlink():link.symlink_to(destination)
    print('Local toolchain ready; no system package, USB access rule or firmware changed.')


if __name__=='__main__':main()
