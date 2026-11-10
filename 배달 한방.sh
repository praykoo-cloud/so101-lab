#!/usr/bin/env bash
# Pi에서 배달 한방 메뉴를 엽니다. 자세 티칭은 메뉴 3번을 고르세요.
cd -- "$(dirname -- "$0")" || exit 1
export PYTHONUTF8=1
exec .venv/bin/python -m mission.menu "$@"
