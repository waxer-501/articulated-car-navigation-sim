#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
/usr/bin/python3 -m venv --without-pip .venv
if ! .venv/bin/python -m pip --version >/dev/null 2>&1; then
  /usr/bin/python3 -m pip --isolated install --target .venv/lib/python3.10/site-packages pip==26.2.2
fi
.venv/bin/python -m pip install -r requirements.lock
.venv/bin/python -m pip install --no-deps -e .
mkdir -p .runtime/packages
if [[ ! -f .runtime/usr/lib/x86_64-linux-gnu/libxcb-cursor.so.0 ]]; then
  (cd .runtime/packages && apt-get download libxcb-cursor0=0.1.1-4ubuntu1)
  dpkg-deb -x .runtime/packages/libxcb-cursor0_0.1.1-4ubuntu1_amd64.deb .runtime
fi
