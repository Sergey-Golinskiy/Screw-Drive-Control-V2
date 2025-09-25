#!/usr/bin/env bash
set -euo pipefail
# Подчищаем fbi, если висит
if pgrep -x fbi >/dev/null 2>&1; then
  pkill -x fbi || true
fi
# Сброс и очистка tty1
exec </dev/tty1 >/dev/tty1 2>&1
printf '\033c'
clear || true