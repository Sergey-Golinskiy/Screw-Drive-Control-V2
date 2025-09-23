import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
cmd  = sys.argv[2] if len(sys.argv) > 2 else "M119"
baud = 115200


def open_noreset(p, b):
    # открываем порт без аппаратного флоуконтроля и без ресета
    ser = serial.Serial(
        port=p,
        baudrate=b,
        timeout=0.2,
        write_timeout=0.2,
        rtscts=False,
        dsrdtr=False
    )
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass

    # очистка буферов
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def drain_until_ready(ser, wait_s=2.0):
    """Ждем баннер 'ok READY' после возможного ресета (не обязательно, но безопасно)."""
    t0 = time.time()
    got_any = False
    while time.time() - t0 < wait_s:
        ln = ser.readline()
        if not ln:
            continue
        s = ln.decode(errors="ignore").strip()
        if s:
            got_any = True
            # распечатаем всё, что пришло (на всякий)
            # print(s)   # при желании закомментируй/раскомментируй
            if s.startswith("ok"):
                # это может быть 'ok READY' или просто 'ok'
                break
    # Если ничего не пришло — ок, просто идём дальше.

def main():
    # На всякий: один раз после ребута можно выполнить
    #   stty -F /dev/ttyACM0 -hupcl
    # чтобы ядро не дёргало DTR при close/open.

    ser = open_noreset(port, baud)

    # Если был ресет — ждём баннер; если нет — быстро пройдём цикл
    drain_until_ready(ser, wait_s=2.0)

    # Отправляем команду
    line = (cmd.strip() + "\n").encode()
    ser.write(line)

    # Читаем ответ до 'ok' (или 2 сек таймаут)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        ln = ser.readline()
        if not ln:
            continue
        s = ln.decode(errors="ignore").strip()
        if s:
            print(s)
            if s == "ok" or s.startswith("ok "):
                break

    ser.close()

if __name__ == "__main__":
    # Совет: один раз отключи автосброс на уровне драйвера:
    #   stty -F /dev/ttyACM0 -hupcl
    # и (опционально) удалите ModemManager/BRLTTY, если стоят.
    main()
