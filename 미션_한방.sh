#!/usr/bin/env bash
# 로봇팔 미션 한방 메뉴 (Pi · 리눅스).  사용:  ./미션_한방.sh   또는   ./미션_한방.sh --fake
export PYTHONUTF8=1
cd "$(dirname "$0")"
if [ -x .venv/bin/python ]; then PY=.venv/bin/python; else PY=python3; fi
exec "$PY" -m mission.menu "$@"
