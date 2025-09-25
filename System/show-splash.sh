#!/usr/bin/env bash
set -euo pipefail
# Привязываем ввод/вывод к tty1 и показываем PNG через fbi
exec </dev/tty1 >/dev/tty1 2>&1
if [ -e /dev/fb0 ]; then
  FRAMEBUFFER=/dev/fb0 /usr/bin/fbi -T 1 -d /dev/fb0 --noverbose -a /opt/splash/splash.png
fi