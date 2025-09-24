#!/usr/bin/env python3
# -*- coding: utf-8 -*-
TRIGGER_HOST = "127.0.0.1"
TRIGGER_PORT = 8765

import time
import threading
from datetime import datetime
import os
from pathlib import Path
import argparse, json
from pathlib import Path
import yaml

from typing import Optional
import RPi.GPIO as GPIO

# ===[ ДОБАВЛЕНО: serial ]===
import serial
try:
    import serial
except Exception:
    serial = None

import socket
try:
    import socket
except Exception:
    socket = None
# =====================[ КОНФИГ ]=====================
RELAY_ACTIVE_LOW = True  # твоя 8-релейка, как правило, LOW-trigger
BUSY_FLAG = "/tmp/screw_cycle_busy"

DEFAULT_CFG = Path(__file__).with_name("devices.yaml")
TASK_PULSE_MS = 700  # п.7: импульс выбора задачи

# Файл конфигурации устройств (датчиков/реле)
# Реле (BCM): подгони под свою распиновку при необходимости
RELAY_PINS = {
    "R01_PIT":     5,   # Питатель винтов (импульс)
    "R04_C2":      16,  # Цилиндр отвёртки (ON=вниз, OFF=вверх)
    "R05_DI4_FREE":  19,  # Свободный ход отвёртки (держать ON = крутится)
    "R06_DI1_POT":   20,  # Режим «по моменту» (держать ON до ОК)
    "R07_DI5_TSK0":  21,  # Выбор задачи 0 (импульс 700 мс)
    "R08_DI6_TSK1":  26,  # Выбор задачи 1 (импульс 700 мс)
}

# BCM-распиновка датчиков (герконов). True=CLOSE (замкнут на GND)
SENSOR_PINS = {
    #27
    "GER_C2_UP":    22,  # верх отвёртки
    "GER_C2_DOWN":  23,  # низ отвёртки
    "IND_SCRW":     12,  # <— ИНДУКТИВНЫЙ: «винт прошёл»
    "DO2_OK":       25,  # Ответ от драйвера что винт закручен с нудным моментом, успех
    "PED_START":    18,  # педалька для старта цикла
    "AREA_SENSOR":  17,  # Штора безопасности. сенсор OPEN=преград нет, все ок, CLOSE=преграда, стоп
}

# Парные взаимоблокировки (оставили как в базе; сейчас R02/R03 не конфликтуют, но пусть будет)
MUTEX_GROUPS = [
    ("R02_C1_UP", "R03_C1_DOWN"),
]

SENSOR_BOUNCE_MS = 20
POLL_INTERVAL_MS = 5

# Таймауты/времена
TIMEOUT_SEC = 2.0                 # ожидания герконов/датчиков (кроме педали)
FEED_PULSE_MS = 200               # п.9/16/23: импульс подачі
IND_PULSE_WINDOW_MS = 1000         # п.10/17/24: окно контроля IND_SCRW
FREE_BURST_MS = 100               # п.14/21/28: импульс free-run
MOVE_F = 30000                    # скорость G-команд

# Точки для трёх подач (п.8, п.15, п.22)
POINTS = [
    (35, 153),
    (15, 123),
    (54, 123),
]

# Серийный порт
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 0.5
SERIAL_WTIMEOUT = 0.5


def set_cycle_busy(on: bool):
    try:
        if on:
            Path(BUSY_FLAG).write_text("1")
        else:
            try:
                os.remove(BUSY_FLAG)
            except FileNotFoundError:
                pass
    except Exception:
        pass

def is_port_open(host="127.0.0.1", port=8765, timeout=0.2) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False
# =====================[ ВСПОМОГАТЕЛЬНОЕ ]=====================
def ts():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def relay_gpio_value(on: bool) -> int:
    return GPIO.LOW if (RELAY_ACTIVE_LOW and on) or ((not RELAY_ACTIVE_LOW) and (not on)) else GPIO.HIGH

# =====================[ IO КОНТРОЛЛЕР ]=======================
class IOController:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        for pin in RELAY_PINS.values():
            GPIO.setup(pin, GPIO.OUT, initial=relay_gpio_value(False))

        for pin in SENSOR_PINS.values():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.relays = {name: False for name in RELAY_PINS.keys()}

        # Попытка повесить edge; если не выйдет — polling fallback
        self._use_poll_fallback = False
        self._last_state = {}
        edge_ok = True
        for name, pin in SENSOR_PINS.items():
            try:
                try:
                    GPIO.remove_event_detect(pin)
                except Exception:
                    pass
                GPIO.add_event_detect(pin, GPIO.BOTH, callback=self._sensor_event, bouncetime=SENSOR_BOUNCE_MS)
                self._last_state[name] = (GPIO.input(pin) == GPIO.LOW)
            except Exception as e:
                print(f"[{ts()}] WARN: Edge detect failed on {name} (GPIO{pin}): {e}")
                edge_ok = False

        if not edge_ok:
            print(f"[{ts()}] INFO: Switching to polling fallback for sensors.")
            self._use_poll_fallback = True
            for name, pin in SENSOR_PINS.items():
                self._last_state[name] = (GPIO.input(pin) == GPIO.LOW)
            self._poll_stop = threading.Event()
            self._poll_thr = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll_thr.start()

        print(f"[{ts()}] IO init done. All relays OFF. Edge={'ON' if not self._use_poll_fallback else 'OFF/Polling'}")

    def cleanup(self):
        if getattr(self, "_use_poll_fallback", False):
            self._poll_stop.set()
            if hasattr(self, "_poll_thr"):
                self._poll_thr.join(timeout=0.5)
        # выключать реле при выходе — по ситуации; оставим безопасно OFF
        for name in list(self.relays.keys()):
            self._apply_relay(name, False)
        GPIO.cleanup()

    # ---- Реле
    def _apply_relay(self, relay_name: str, on: bool):
        pin = RELAY_PINS[relay_name]
        GPIO.output(pin, relay_gpio_value(on))
        self.relays[relay_name] = on
        print(f"[{ts()}] {relay_name} -> {'ON' if on else 'OFF'}")

    def set_relay(self, relay_name: str, on: bool):
        if relay_name not in RELAY_PINS:
            raise ValueError(f"Unknown relay '{relay_name}'")
        if on:
            for a, b in MUTEX_GROUPS:
                if relay_name == a and self.relays.get(b, False):
                    self._apply_relay(b, False)
                if relay_name == b and self.relays.get(a, False):
                    self._apply_relay(a, False)
        self._apply_relay(relay_name, on)

    def pulse(self, relay_name: str, ms: int):
        self.set_relay(relay_name, True)
        time.sleep(ms / 1000.0)
        self.set_relay(relay_name, False)

    # ---- Датчики
    def sensor_state(self, sensor_name: str) -> bool:
        pin = SENSOR_PINS[sensor_name]
        return GPIO.input(pin) == GPIO.LOW  # True=CLOSE

    def _sensor_event(self, ch_pin: int):
        # просто печать изменений; логика цикла опрашивает синхронно
        for n, p in SENSOR_PINS.items():
            if p == ch_pin:
                print(f"[{ts()}] SENSOR {n}: {'CLOSE' if self.sensor_state(n) else 'OPEN'}")
                break

    def _poll_loop(self):
        stable_required = max(1, SENSOR_BOUNCE_MS // POLL_INTERVAL_MS)
        counters = {name: 0 for name in SENSOR_PINS.keys()}
        while not self._poll_stop.is_set():
            for name, pin in SENSOR_PINS.items():
                closed_now = (GPIO.input(pin) == GPIO.LOW)
                if closed_now != self._last_state[name]:
                    counters[name] += 1
                    if counters[name] >= stable_required:
                        self._last_state[name] = closed_now
                        counters[name] = 0
                        print(f"[{ts()}] SENSOR {name}: {'CLOSE' if closed_now else 'OPEN'}")
                else:
                    counters[name] = 0
            time.sleep(POLL_INTERVAL_MS / 1000.0)

# =====================[ SERIAL / G-КОД ]=======================
def open_serial():
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUD,
        timeout=SERIAL_TIMEOUT,
        write_timeout=SERIAL_WTIMEOUT,
        rtscts=False,
        dsrdtr=False
    )
    # Отключаем автосбросные линии
    ser.dtr = False
    ser.rts = False
    return ser

def wait_ready(ser: serial.Serial, timeout: float = 5.0) -> bool:
    """
    Ждём строку 'ok READY' от прошивки Arduino.
    Возвращает True при успехе, False при таймауте.
    """
    t_end = time.time() + timeout
    while time.time() < t_end:
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        # допускаем разные регистры/пробелы
        if s.lower().replace("  ", " ").strip() == "ok ready":
            return True
    print("[SER] TIMEOUT: не получили 'ok READY'")
    return False


def send_cmd(ser: serial.Serial, line: str):
    """Отправить команду и дождаться ok/err; печатаем ответы."""
    payload = (line.strip() + "\n").encode()
    ser.write(payload)
    while True:
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        if s.startswith("ok") or s.startswith("err"):
            break

def move_xy(ser: serial.Serial, x: float, y: float, f: int = MOVE_F):
    send_cmd(ser, f"G X{x} Y{y} F{f}")

# =====================[ ХЕЛПЕРЫ ЛОГИКИ ]=======================
def wait_sensor(io: IOController, sensor_name: str, target_close: bool, timeout: float | None) -> bool:
    start = time.time()
    wanted = "CLOSE" if target_close else "OPEN"
    while True:
        if io.sensor_state(sensor_name) == target_close:
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print(f"[wait_sensor] TIMEOUT: {sensor_name} не достиг состояния {wanted} за {timeout} с")
            return False
        time.sleep(0.01)

def wait_new_press(io: IOController, sensor_name: str, timeout: float | None) -> bool:
    """Ждём новую нажим педали (OPEN -> CLOSE)"""
    start = time.time()
    # дождаться OPEN
    while True:
        if not io.sensor_state(sensor_name):  # OPEN
            break
        if timeout is not None and (time.time() - start) > timeout:
            print(f"[wait_new_press] TIMEOUT: {sensor_name} не вернулась в OPEN")
            return False
        time.sleep(0.01)
    # дождаться CLOSE
    start = time.time()
    while True:
        if io.sensor_state(sensor_name):  # CLOSE
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print(f"[wait_new_press] TIMEOUT: {sensor_name} не нажата")
            return False
        time.sleep(0.01)

class StartTrigger:
    def __init__(self, host: str = TRIGGER_HOST, port: int = TRIGGER_PORT):
        self.host = host
        self.port = port
        self.event = threading.Event()
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None

    def start(self):
        if self._thr and self._thr.is_alive():
            return
        self._thr = threading.Thread(target=self._server_loop, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop.set()
        # разбудить accept()
        try:
            with socket.create_connection((self.host, self.port), timeout=0.2):
                pass
        except Exception:
            pass
        if self._thr:
            self._thr.join(timeout=0.5)

    def trigger_once(self):
        self.event.clear()

    def _server_loop(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            s.settimeout(0.5)
            print(f"[trigger] LISTEN {self.host}:{self.port}")
        except Exception as e:
            print(f"[trigger] LISTEN FAILED on {self.host}:{self.port}: {e}")
            return

        while not self._stop.is_set():
            try:
                conn, _ = s.accept()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[trigger] accept error: {e}")
                continue

            with conn:
                try:
                    data = conn.recv(64)
                    if data and b"START" in data.upper():
                        print("[trigger] Получена команда START от UI")
                        self.event.set()
                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"ERR\n")
                except Exception as e:
                    print(f"[trigger] recv/send error: {e}")
        try:
            s.close()
        except Exception:
            pass
        print("[trigger] listener stopped")



def wait_pedal_or_command(io: IOController, trg: "StartTrigger" | None = None) -> bool:
    # первая фаза — дождаться OPEN (если педаль держат)
    while True:
        if not io.sensor_state("PED_START"):
            break
        if trg and trg.event.is_set():
            trg.trigger_once()
            return True
        time.sleep(0.01)
    # вторая фаза — дождаться CLOSE
    while True:
        if io.sensor_state("PED_START"):
            return True
        if trg and trg.event.is_set():
            trg.trigger_once()
            return True
        time.sleep(0.01)



def wait_close_pulse(io: IOController, sensor_name: str, window_ms: int) -> bool:
    """Ждём, что датчик станет CLOSE хотя бы импульсно в течение window_ms."""
    t_end = time.time() + (window_ms / 1000.0)
    while time.time() < t_end:
        if io.sensor_state(sensor_name): 
            return True
        time.sleep(0.005)
    return False

def feed_until_detect(io: IOController) -> bool:
    while True:
        io.pulse("R01_PIT", ms=FEED_PULSE_MS)
        if wait_close_pulse(io, "IND_SCRW", IND_PULSE_WINDOW_MS):
            return True
        print("[feed] Нет импульса IND_SCRW, повторяю подачу...")

def torque_sequence(io: IOController) -> bool:
    """
    Включить моментный режим и опустить отвёртку до DO2_OK=CLOSE,
    затем поднять (ждать GER_C2_UP) и дать free-run импульс.
    Возвращает True при успехе, False при таймауте (в этом случае всё выключено и инструмент поднят).
    """
    io.set_relay("R06_DI1_POT", True)           # п.11 / 18 / 25
    io.set_relay("R04_C2", True)                # п.12 / 19 / 26
    ok = wait_sensor(io, "DO2_OK", True, TIMEOUT_SEC)
    if not ok:
        print("[torque] TIMEOUT по DO2_OK — выключаю и поднимаю C2")
        io.set_relay("R04_C2", False)
        io.set_relay("R06_DI1_POT", False)
        wait_sensor(io, "GER_C2_UP", True, TIMEOUT_SEC)
        return False

    # момент достигнут — поднять инструмент
    io.set_relay("R04_C2", False)               # п.13 / 20 / 27 (часть 1)
    io.set_relay("R06_DI1_POT", False)          # п.13 / 20 / 27 (часть 2)
    ok_up = wait_sensor(io, "GER_C2_UP", True, TIMEOUT_SEC)
    if not ok_up:
        return False

    # free-run импульс 100 мс (п.14 / 21 / 28)
    io.pulse("R05_DI4_FREE", ms=FREE_BURST_MS)
    return True

def torque_fallback(io: IOController):
    """
    Аварийный вариант: момент не достигнут.
    Просто поднимаем отвёртку (GER_C2_UP) и выключаем реле.
    """
    io.set_relay("R04_C2", False)
    io.set_relay("R06_DI1_POT", False)
    wait_sensor(io, "GER_C2_UP", True, TIMEOUT_SEC)

def load_devices_config(path: Path = DEFAULT_CFG) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not data or "devices" not in data:
        raise ValueError("devices.yaml: не найден список devices")
    # простая валидация
    by_key = {}
    for d in data["devices"]:
        if "key" not in d or "name" not in d or "program" not in d:
            raise ValueError(f"devices.yaml: device не полон: {d}")
        by_key[d["key"]] = d
    return by_key

def select_task(io: "IOController", task: int, ms: int = TASK_PULSE_MS):
    if task == 0:
        io.pulse("R07_DI5_TSK0", ms=ms)
    elif task == 1:
        io.pulse("R08_DI6_TSK1", ms=ms)
    else:
        print(f"[task] unknown task={task}, пропускаю")

def run_cycle(selected_key: str):
    # 1) выбрать устройство и программу
    devices = load_devices_config()
    dev = devices.get(selected_key)
    if not dev:
        print(f"[err] device '{selected_key}' не найден в devices.yaml")
        return 1

    program = dev["program"]
    print(f"[info] Устройство: {dev['name']} (holes={dev.get('holes')})")

    # 2) обычная твоя инициализация: GPIO, serial, ready, homing и т.д.
    io = IOController()
    ser = open_serial()
    if not wait_ready(ser):
        print("[err] контроллер перемещений не готов")
        return 2
    send_cmd(ser, "G28")  # хоуминг, как у тебя

    # 3) ожидание старта (педаль/команда START)
    if not wait_pedal_or_command(io):   # твоя функция
        return 3

    # 4) основной маршрут по шагам
    for step in program:
        t = step.get("type", "free")
        x = step["x"]; y = step["y"]
        f = step.get("f")

        move_xy(ser, x, y, f)  # твоя функция

        if t == "work":
            # (опционально) выбрать таску
            if "task" in step:
                select_task(io, int(step["task"]))

            # подача винта до импульса IND_SCRW (твоя функция)
            feed_until_detect(io)

            # закручивание по моменту (твоя функция)
            if not torque_sequence(io):
                print("[err] момент не достигнут — аварийный выход")
                move_xy(ser, 35, 20)  # безопасная позиция
                break
        # 'free' — просто проезд

    # 5) финализация/очистка если нужно
    try:
        io.cleanup()
    except Exception:
        pass
    return 0


# =====================[ ГЛАВНАЯ ЛОГИКА ]=======================
def main():
    # 0) выбрать устройство из конфигурации (требуем аргумент)
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True, help="device key from devices.yaml")
    args = parser.parse_args()

    devices = load_devices_config()
    dev = devices.get(args.device)
    if not dev:
        print(f"[err] device '{args.device}' не найден в devices.yaml")
        raise SystemExit(1)
    program = dev["program"]
    print(f"[info] Устройство: {dev['name']} (holes={dev.get('holes')})")

    # 1) инициализация GPIO и триггера
    io = IOController()
    trg = StartTrigger(TRIGGER_HOST, TRIGGER_PORT)
    trg.start()

    # 2) открыть serial и проверить готовность контроллера
    print(f"[{ts()}] Открываю сериал порт {SERIAL_PORT} @ {SERIAL_BAUD}")
    ser = open_serial()
    print(f"[{ts()}] Serial открыт")
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    if not wait_ready(ser, timeout=5.0):
        print("[err] контроллер перемещений не готов (нет 'ok READY')")
        trg.stop()
        io.cleanup()
        try: ser.close()
        except Exception: pass
        raise SystemExit(2)

    # 3) базовая инициализация координатной системы
    print("=== Старт скрипта ===")
    send_cmd(ser, "G28")   # хоуминг
    send_cmd(ser, "WORK")  # привести механику в безопасный «рабочий» пресет

    # локальный хелпер перемещения (если у тебя есть глобальный move_xy — можешь использовать его)
    def _move_xy(ser_, x, y, f=None):
        feed = f or MOVE_F
        send_cmd(ser_, f"G X{x} Y{y} F{feed}")

    try:
        while True:
            # ——— ожидание запуска ———
            set_cycle_busy(False)
            print("[cycle] Жду педаль PED_START ИЛИ команду START от UI...")
            if not wait_pedal_or_command(io, trg):
                print("[cycle] Старт не получен — выхожу")
                break

            set_cycle_busy(True)

            # ——— исполнение маршрута из devices.yaml ———
            abort = False
            for step in program:
                t = step.get("type", "free")
                x = step["x"]; y = step["y"]; f = step.get("f")

                # (опционально) safety: шторка
                if io.sensor_state("AREA_SENSOR"):
                    print("[safety] AREA_SENSOR=CLOSE — зона занята! Прерываю цикл.")
                    io.set_relay("R04_C2", False)      # поднять цилиндр
                    io.set_relay("R06_DI1_POT", False) # снять момент
                    # ждать вверх по возможности, но без паники, затем вернуть систему WORK
                    try: wait_sensor(io, "GER_C2_UP", True, 2.0)
                    except Exception: pass
                    send_cmd(ser, "WORK")
                    abort = True
                    break

                # перемещение
                _move_xy(ser, x, y, f)

                if t == "work":
                    # (необ.) импульс выбора таски
                    if "task" in step:
                        select_task(io, int(step["task"]))  # 0 -> R07, 1 -> R08

                    # подача винта до импульса IND_SCRW
                    if not feed_until_detect(io):
                        print("[feed] Нет импульса IND_SCRW в окне — аварийный выход")
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                    # закрутка по моменту: R06/R04 → DO2_OK → вверх → free-run
                    if not torque_sequence(io):
                        print("[torque] Момент не достигнут — аварийный выход")
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                # небольшая тех.пауза (сгладить дребезг статусов)
                time.sleep(0.05)

            # ——— окончание серии ———
            send_cmd(ser, "WORK")      # привести механику в безопасный рабочий пресет
            set_cycle_busy(False)

            if abort:
                print("[cycle] Серия прервана — ожидаю новый старт")
            else:
                print("[cycle] Серия шагов завершена — ожидаю новый старт")

            # цикл продолжается: снова ждём педаль/команду
            # если нужен одноразовый запуск — раскомментируй:
            # break

    except KeyboardInterrupt:
        print("[cycle] Прервано пользователем (Ctrl+C)")
    finally:
        trg.stop()
        try:
            # безопасно снять выходы
            io.set_relay("R04_C2", False)
            io.set_relay("R06_DI1_POT", False)
        except Exception:
            pass
        try:
            set_cycle_busy(False)
        except Exception:
            pass
        try:
            io.cleanup()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("=== Остановлено. GPIO освобождены, serial закрыт ===")

if __name__ == "__main__":
    main()
