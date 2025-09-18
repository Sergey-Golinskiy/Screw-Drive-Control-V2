#!/usr/bin/env bash
# 1) попробовать очистить TTY1 (на случай текстового вывода)
printf "\033c" > /dev/tty1 2>/dev/null || true
# 2) перерисовать экран чёрным (надёжно для framebuffer)
if [ -e /dev/fb0 ]; then
  if [ ! -f /opt/splash/black.png ]; then
    # создадим чёрную картинку 1920x1080 если её нет
    command -v convert >/dev/null && convert -size 1920x1080 xc:black /opt/splash/black.png
  fi
  if [ -f /opt/splash/black.png ]; then
    FRAMEBUFFER=/dev/fb0 /usr/bin/fbi -a -T 1 -d /dev/fb0 --noverbose /opt/splash/black.png </dev/tty1 >/dev/tty1 2>/dev/null || true
  fi
fi
