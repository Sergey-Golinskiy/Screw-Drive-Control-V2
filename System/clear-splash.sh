#!/usr/bin/env bash
printf "\033c" > /dev/tty1 2>/dev/null || true
if [ -e /dev/fb0 ]; then
  if [ ! -f /opt/splash/black.png ]; then
    convert -size 1920x1080 xc:black /opt/splash/black.png
  fi
  FRAMEBUFFER=/dev/fb0 /usr/bin/fbi -a -T 1 -d /dev/fb0 --noverbose /opt/splash/black.png </dev/tty1 >/dev/tty1 2>/dev/null || true
fi