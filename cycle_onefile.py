#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import time
import threading
from datetime import datetime
import os
from pathlib import Path
import argparse, json
import yaml
from collections import deque
import threading
import json
from typing import Optional
import RPi.GPIO as GPIO
# ===[ ДОБАВЛЕНО: serial ]===
import serial

import socket

# =====================[ ТРИГГЕР ]====================
TRIGGER_HOST = "127.0.0.1"
TRIGGER_PORT = 8765

# =====================[ КОНФИГ ]=====================
RELAY_ACTIVE_LOW = True  # твоя 8-релейка, как правило, LOW-trigger
BUSY_FLAG = "/tmp/screw_cycle_busy"
LAST_EXIT_PATH = Path("/tmp/last_exit.json")
DEFAULT_CFG = Path(__file__).with_name("devices.yaml")
TASK_PULSE_MS = 700  # п.7: импульс выбора задачи
UI_STATUS_PATH = Path("/tmp/ui_status.json")
EVENT_LOG_PATH = Path("/tmp/screw_events.jsonl")
EVENT_BUFFER_SIZE = 200  # сколько последних событий держать в памяти

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

class StatusBus:
    def __init__(self, path: Path):
        self.path = path
        self.buf = deque(maxlen=EVENT_BUFFER_SIZE)
        self.lock = threading.Lock()

    def emit(self, code: str, level: str, msg: str, **extra):
        evt = {
            "ts": ts(),         # твоя функция формата времени
            "code": code,       # короткий код события
            "level": level,     # INFO|WARN|ERROR|ALARM
            "msg": msg,         # человекочитаемое сообщение
            "extra": extra or {}
        }
        with self.lock:
            self.buf.append(evt)
            try:
                with self.path.open("a", encoding="utf-8") as f:
                    f.write(json.dumps(evt, ensure_ascii=False) + "\n")
            except Exception:
                pass  # не валим цикл, если диск недоступен

# глобальный экземпляр
STATUS = StatusBus(EVENT_LOG_PATH)

# удобные хэлперы
def ev_info(code, msg, **kw):  STATUS.emit(code, "INFO",  msg, **kw)
def ev_warn(code, msg, **kw):  STATUS.emit(code, "WARN",  msg, **kw)
def ev_err(code, msg, **kw):   STATUS.emit(code, "ERROR", msg, **kw)
def ev_alarm(code, msg, **kw): STATUS.emit(code, "ALARM", msg, **kw)


def ui_status_update(**kv):
    """Безопасно пишет статус для UI (одно место для всех статусов)."""
    base = {"status_text": "", "can_tighten": False, "phase": ""}  # можно расширять
    try:
        if UI_STATUS_PATH.exists():
            base.update(json.loads(UI_STATUS_PATH.read_text()))
    except Exception:
        pass
    base.update(kv)
    UI_STATUS_PATH.write_text(json.dumps(base, ensure_ascii=False))

def write_exit_reason(code: str, msg: str, extra: dict | None = None):
    data = {"ts": ts(), "code": code, "msg": msg, "extra": extra or {}}
    try:
        with LAST_EXIT_PATH.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
    except Exception as e:
        print(f"[{ts()}] WARN: cannot write {LAST_EXIT_PATH}: {e}")


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

def handle_area_trip(io: "IOController", ser) -> None:
    # немедленно снять усилия/опускание
    try:
        io.set_relay("R06_DI1_POT", False)  # момент
        io.set_relay("R04_C2", False)       # цилиндр вверх
    except Exception:
        pass
    try:
        wait_sensor(io, "GER_C2_UP", True, 2.0)
    except Exception:
        pass
    # перевести механику в безопасный пресет контроллера
    send_cmd(ser, "WORK")
    # событие
    try:
        ev_alarm("AREA_TRIP", "Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.")
    except Exception:
        print("[safety] AREA_SENSOR TRIP: Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.")


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
                print(f"[{ts()}] ПОПЕРЕДЖЕННЯ: збоїло виявлення edge на {name} (GPIO{pin}): {e}")
                edge_ok = False

        if not edge_ok:
            print(f"[{ts()}] ІНФО: Перехід на опитування (polling) датчиків.")
            self._use_poll_fallback = True
            for name, pin in SENSOR_PINS.items():
                self._last_state[name] = (GPIO.input(pin) == GPIO.LOW)
            self._poll_stop = threading.Event()
            self._poll_thr = threading.Thread(target=self._poll_loop, daemon=True)
            self._poll_thr.start()

        print(f"[{ts()}] Ініціалізацію IO виконано. Усі реле OFF. Edge={'ON' if not self._use_poll_fallback else 'OFF/Polling'}")

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
                print(f"[{ts()}] ДАТЧИК {n}: {'CLOSE' if self.sensor_state(n) else 'OPEN'}")
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
    print("[SER] TIMEOUT: не отримали 'ok READY'")
    return False

def home_to_zero(ser: serial.Serial, timeout: float = 30.0) -> bool:
    """
    Отправляем 'G28' и ждём:
      1) 'IN_HOME_POS' (фактический выезд в нули),
      2) затем 'ok' (успешное завершение).
    Если видим 'HOME_NOT_FOUND' или 'err ...' — считаем ошибкой и НЕ продолжаем.
    """
    # на всякий случай очистим входной буфер
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # отправить G28
    ser.write(b"G28\n")
    t_end = time.time() + timeout
    got_in_home = False

    while time.time() < t_end:
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        ls = s.lower()

        if "home_not_found" in ls:
            ev_err("HOME_NOT_FOUND", "Контролер не знайшов домашню позицію (HOME_NOT_FOUND)", popup=True)
            return False

        if "in_home_pos" in ls and not got_in_home:
            got_in_home = True
            ev_info("IN_HOME_POS", "Стіл у нульових координатах")
            # продолжаем читать до финального ok

        if s.startswith("err"):
            ev_err("HOME_ERR", f"Помилка під час G28: {s}", popup=True)
            return False

        # успешное завершение — только когда уже был IN_HOME_POS и пришло 'ok'
        if s.startswith("ok") and got_in_home:
            ev_info("HOME_OK", "Хоумінг завершено (IN_HOME_POS → ok)")
            return True

    ev_err("HOME_TIMEOUT", f"Не дочекалися IN_HOME_POS/ok за {timeout}s", popup=True)
    return False


def send_cmd(ser: serial.Serial, line: str):
    """Отправить команду и дождаться ok/err; печатаем ответы."""
    payload = (line.strip() + "\n").encode()
    ser.reset_input_buffer()
    ser.write(payload)
    while True:
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        if s.startswith("ok") or s.startswith("err"):
            break

def send_cmd_collect(ser: serial.Serial, line: str, timeout: float = 5.0) -> list[str]:
    payload = (line.strip() + "\n").encode()
    ser.reset_input_buffer()
    ser.write(payload)
    lines: list[str] = []
    t0 = time.time()
    while True:
        if time.time() - t0 > timeout:
            raise TimeoutError(f"Timeout waiting OK for '{line}'")
        s = ser.readline().decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        lines.append(s)
        if s.startswith("ok") or s.startswith("err"):
            return lines

def move_xy(ser: serial.Serial, x: float, y: float, f: int = MOVE_F):
    send_cmd(ser, f"G X{x} Y{y} F{f}")

def _parse_status_xy(lines: list[str]) -> tuple[float | None, float | None]:
    x_val: float | None = None
    y_val: float | None = None
    for ln in lines:
        if ln.startswith("STATUS "):
            parts = ln.split()
            for p in parts:
                if p.startswith("X:"):
                    x_val = float(p[2:])
                elif p.startswith("Y:"):
                    y_val = float(p[2:])
            break
    return x_val, y_val

def get_position_mm(ser: serial.Serial) -> tuple[float | None, float | None]:
    try:
        lines = send_cmd_collect(ser, "M114", timeout=3.0)
    except Exception as e:
        return None, None
    return _parse_status_xy(lines)

def verify_position_reached(ser: serial.Serial, target_x: float, target_y: float, tol_mm: float = 0.30) -> tuple[bool, dict]:
    x_mm, y_mm = get_position_mm(ser)
    if x_mm is None or y_mm is None:
        return False, {"reason": "no_status"}
    dx = abs(x_mm - float(target_x))
    dy = abs(y_mm - float(target_y))
    ok = (dx <= tol_mm) and (dy <= tol_mm)
    return ok, {"x": x_mm, "y": y_mm, "dx": dx, "dy": dy, "tol": tol_mm}

# =====================[ ХЕЛПЕРЫ ЛОГИКИ ]=======================
def wait_sensor(io: IOController, sensor_name: str, target_close: bool, timeout: float | None) -> bool:
    start = time.time()
    wanted = "CLOSE" if target_close else "OPEN"
    while True:
        if io.sensor_state(sensor_name) == target_close:
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print(f"[wait_sensor] TIMEOUT: {sensor_name} не досяг стану {wanted} за {timeout} с")
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
            print(f"[wait_new_press] TIMEOUT: {sensor_name} не повернувся в OPEN")
            return False
        time.sleep(0.01)
    # дождаться CLOSE
    start = time.time()
    while True:
        if io.sensor_state(sensor_name):  # CLOSE
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print(f"[wait_new_press] TIMEOUT: {sensor_name} не натиснута")
            return False
        time.sleep(0.01)

def check_pneumatics_start(io: "IOController") -> bool:
    """
    1.1 Перед открытием serial:
        GER_C2_UP == CLOSE (True) и GER_C2_DOWN == OPEN (False).
        Если не так — popup в UI и фиксируем причину, далее выходим из main().
    """
    up = io.sensor_state("GER_C2_UP")      # True = CLOSE
    down = io.sensor_state("GER_C2_DOWN")  # True = CLOSE

    if up is True and down is False:
        ui_status_update(status_text="Стартові стани валідні (UP=CLOSE, DOWN=OPEN)", can_tighten=False, phase="ready_to_start_ok_pneumatics")
        ev_info("PNEUM_START_OK", "Стартові стани валідні (UP=CLOSE, DOWN=OPEN)")
        return True

    msg = ("Перевірте пневматику: на старті очікуємо GER_C2_UP=CLOSE, "
           "GER_C2_DOWN=OPEN. Скрипт зупинено.")
    ev_err("PNEUMATICS_NOT_READY", msg, popup=True,
           expect_up=True, expect_down=False, actual_up=up, actual_down=down)
    write_exit_reason("pneumatics_not_ready", msg,
                      {"expect_up": True, "expect_down": False,
                       "actual_up": up, "actual_down": down})
    return False

def cyl_down_up(io: "IOController", timeout: float = TIMEOUT_SEC) -> bool:
    """
    1.2 Включаем R04_C2 (вниз) до GER_C2_DOWN=CLOSE, затем выключаем (вверх)
        до GER_C2_UP=CLOSE.
    """
    # ВНИЗ
    io.set_relay("R04_C2", True)
    if not wait_sensor(io, "GER_C2_DOWN", True, timeout):
        io.set_relay("R04_C2", False)
        msg = f"Не досягли GER_C2_DOWN=CLOSE за {timeout}s"
        ev_err("C2_NO_DOWN", msg, popup=True)
        write_exit_reason("c2_no_down", msg, {})
        return False

    # ВВЕРХ
    io.set_relay("R04_C2", False)
    if not wait_sensor(io, "GER_C2_UP", True, timeout):
        msg = f"Не досягли GER_C2_UP=CLOSE за {timeout}s"
        ev_err("C2_NO_UP", msg, popup=True)
        write_exit_reason("c2_no_up", msg, {})
        return False

    ev_info("PNEUM_DU_OK", "Циліндр: вниз/вгору виконано")
    return True

def select_task0(io: "IOController", ms: int = 500) -> None:
    """
    1.3 Імпульс R07_DI5_TSK0 на 500 мс (застосувати таску 0).
    """
    try:
        io.pulse("R07_DI5_TSK0", ms=ms)
        ev_info("TASK0_SELECT", "Застосовано таск 0", ms=ms)
    except Exception as e:
        ev_warn("TASK0_PULSE_FAIL", f"Не вдалося подати імпульс таски 0: {e}")

def wait_motors_ok_and_ready(ser: serial.Serial, timeout: float = 15.0) -> bool:
    """
    Ждём (в любом порядке):
      - 'MOT_X_OK'
      - 'MOT_Y_OK'
      - 'ok READY'
    При 'MOT_X_ALARM'/'MOT_Y_ALARM' отправляем 'MOT_X_RESET'/'MOT_Y_RESET'
    и продолжаем ждать соответствующий '*_OK'.
    """
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    t_end = time.time() + timeout
    got_x_ok = False
    got_y_ok = False
    got_ready = False

    x_alarm_seen = False
    y_alarm_seen = False

    while time.time() < t_end:
        # ✅ Успех проверяем СРАЗУ, до чтения строки
        if got_x_ok and got_y_ok and got_ready:
            return True

        raw = ser.readline()
        if not raw:
            continue  # снова на верх цикла — там ещё раз проверим успех

        s = raw.decode(errors="ignore").strip()
        if not s:
            continue

        print(f"[SER] {s}")
        ls = s.lower().strip()

        # --- ALARM → RESET ---
        if "mot_x_alarm" in ls:
            x_alarm_seen = True
            ev_info("MOT_X_ALARM", "ALARM на осі X — виконуємо скидання")
            try:
                ser.write(b"MOT_X_RESET\n"); ser.flush()
            except Exception as e:
                ev_err("MOT_X_RESET_FAIL", f"Помилка відправки MOT_X_RESET: {e}", popup=True)
                return False
            continue

        if "mot_y_alarm" in ls:
            y_alarm_seen = True
            ev_info("MOT_Y_ALARM", "ALARM на осі Y — виконуємо скидання")
            try:
                ser.write(b"MOT_Y_RESET\n"); ser.flush()
            except Exception as e:
                ev_err("MOT_Y_RESET_FAIL", f"Помилка відправки MOT_Y_RESET: {e}", popup=True)
                return False
            continue

        # --- OK маркеры ---
        if "mot_x_ok" in ls:
            if not got_x_ok:
                got_x_ok = True
                ev_info("MOT_X_OK", "Мотор X готов")
            # ✅ сразу проверим успех
            if got_x_ok and got_y_ok and got_ready:
                return True
            continue

        if "mot_y_ok" in ls:
            if not got_y_ok:
                got_y_ok = True
                ev_info("MOT_Y_OK", "Мотор Y готов")
            if got_x_ok and got_y_ok and got_ready:
                return True
            continue

        if ls == "ok ready":
            if not got_ready:
                got_ready = True
                ev_info("READY_OK", "Контролер готов (ok READY)")
            if got_x_ok and got_y_ok and got_ready:
                return True
            continue

        if ls.startswith("err"):
            ev_err("SER_ERR", f"Контролер відповів помилкою: {s}", popup=True)
            return False

    ev_err(
        "READY_TIMEOUT",
        f"Не отримали всі маркери за {timeout}s "
        f"(X_OK={got_x_ok}, Y_OK={got_y_ok}, READY={got_ready}; "
        f"X_ALARM_SEEN={x_alarm_seen}, Y_ALARM_SEEN={y_alarm_seen})",
        popup=True
    )
    return False


class StartTrigger:
    def __init__(self, host: str = TRIGGER_HOST, port: int = TRIGGER_PORT):
        self.host = host
        self.port = port
        self.event = threading.Event()
        self.reset_event = threading.Event()
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
            print(f"[trigger] СЛУХАЮ {self.host}:{self.port}")
        except Exception as e:
            print(f"[trigger] ПОМИЛКА СТАРТУ СЛУХАННЯ на {self.host}:{self.port}: {e}")
            return

        while not self._stop.is_set():
            try:
                conn, _ = s.accept()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[trigger] помилка accept: {e}")
                continue

            with conn:
                try:
                    data = conn.recv(64)
                    if data and b"START" in data.upper():
                        print("[trigger] Отримано команду START від UI")
                        self.event.set()
                        conn.sendall(b"OK\n")
                    elif b"MOTOR_RESET" in data.upper():
                        print("[trigger] Отримано команду MOTOR_RESET від UI")
                        self.reset_event.set()
                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"ERR\n")
                except Exception as e:
                    print(f"[trigger] помилка recv/send: {e}")
        try:
            s.close()
        except Exception:
            pass
        print("[trigger] слухач зупинений")



def wait_pedal_or_command(io: IOController, trg: StartTrigger | None = None) -> bool:
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
    attempt = 0
    while True:
        attempt += 1
        io.pulse("R01_PIT", ms=FEED_PULSE_MS)
        ev_info("FEED_PULSE", "Імпульс подачі", attempt=attempt, ms=FEED_PULSE_MS)
        if wait_close_pulse(io, "IND_SCRW", IND_PULSE_WINDOW_MS):
            ev_info("FEED_OK", "Гвинт пройшов по IND_SCRW", attempts=attempt, window_ms=IND_PULSE_WINDOW_MS)
            return True
        print("[feed] Немає імпульсу IND_SCRW, повторюю подачу...")
        if attempt >= 5:
            ev_warn("FEED_RETRY", "Немає імпульсу IND_SCRW, повторюємо", attempt=attempt)
        if attempt >= 20:
            ev_alarm("FEED_FAIL", "IND_SCRW не спрацював за ліміт спроб", attempts=attempt)
            return False


def torque_sequence(io: IOController) -> bool:
    ev_info("TORQUE_BEGIN", "Початок закручування за моментом")
    io.set_relay("R06_DI1_POT", True)
    io.set_relay("R04_C2", True)
    t0 = time.time()
    ok_stable_ms = 20
    t_ok = None
    while True:
        if io.sensor_state("GER_C2_DOWN"):
            ev_alarm("C2_DOWN", "Досягнуто нижній кінцевик циліндра")
            ui_status_update(status_text="Девайс не встановлено...", can_tighten=False, phase="error_c2_down")
            io.set_relay("R04_C2", False); io.set_relay("R06_DI1_POT", False)
            wait_sensor(io, "GER_C2_UP", True, 2.0)
            return False
        if io.sensor_state("DO2_OK"):
            if t_ok is None: t_ok = time.time()
            elif (time.time()-t_ok)*1000 >= ok_stable_ms:
                break
        else:
            t_ok = None
        if (time.time()-t0) > TIMEOUT_SEC:
            ev_err("TORQUE_TIMEOUT", f"Момент не досягнуто за {TIMEOUT_SEC}s")
            ui_status_update(status_text="Момент не досягнуто", can_tighten=False, phase="error_torque_timeout")
            io.set_relay("R04_C2", False); io.set_relay("R06_DI1_POT", False)
            wait_sensor(io, "GER_C2_UP", True, 2.0)
            return False
        time.sleep(0.005)

    io.set_relay("R04_C2", False)
    io.set_relay("R06_DI1_POT", False)
    wait_sensor(io, "GER_C2_UP", True, TIMEOUT_SEC)
    io.pulse("R05_DI4_FREE", ms=FREE_BURST_MS)
    ev_info("TORQUE_OK", "Момент досягнуто, інструмент піднято", stable_ms=ok_stable_ms)
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
        raise ValueError("devices.yaml: не знайдено список devices")
    # простая валидация
    by_key = {}
    for d in data["devices"]:
        if "key" not in d or "name" not in d or "program" not in d:
            raise ValueError(f"devices.yaml: опис пристрою неповний: {d}")
        by_key[d["key"]] = d
    return by_key

def select_task(io: "IOController", task: int, ms: int = TASK_PULSE_MS):
    if task == 0:
        io.pulse("R07_DI5_TSK0", ms=ms)
    elif task == 1:
        io.pulse("R08_DI6_TSK1", ms=ms)
    else:
        print(f"[task] невідома task={task}, пропускаю")

def run_cycle(selected_key: str):
    # 1) выбрать устройство и программу
    devices = load_devices_config()
    dev = devices.get(selected_key)
    if not dev:
        print(f"[err] device '{selected_key}' не знайдено в devices.yaml")
        return 1

    program = dev["program"]
    print(f"[info] Пристрій: {dev['name']} (отвори={dev.get('holes')})")

    # 2) обычная твоя инициализация: GPIO, serial, ready, homing и т.д.
    io = IOController()
    ser = open_serial()
    if not wait_ready(ser):
        print("[err] контролер переміщень не готовий")
        ui_status_update(status_text="[err] контролер переміщень не готовий", can_tighten=False, phase="error_no_ready_controller")
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
                ev_info("TASK_SELECT", f"Выбрана таска {int(step['task'])}", task=int(step["task"]))


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

def torque_sequence_with_area(io: "IOController", ser, area_armed: bool) -> bool:
    ev_info("TORQUE_BEGIN", "Початок закручування за моментом")
    io.set_relay("R06_DI1_POT", True)
    io.set_relay("R04_C2", True)
    t0 = time.time()
    ok_stable_ms = 20
    t_ok = None
    while True:
        # авария: нижний конечник
        if io.sensor_state("GER_C2_DOWN"):
            ev_alarm("C2_DOWN", "Досягнуто нижній кінцевик циліндра")
            io.set_relay("R04_C2", False); io.set_relay("R06_DI1_POT", False)
            wait_sensor(io, "GER_C2_UP", True, 2.0)
            send_cmd(ser, "WORK")
            return False

        # контроль шторы (только когда «на охране»)
        if area_armed and io.sensor_state("AREA_SENSOR"):
            handle_area_trip(io, ser)
            return False

        # проверка момента со стабильностью
        if io.sensor_state("DO2_OK"):
            if t_ok is None:
                t_ok = time.time()
            elif (time.time() - t_ok) * 1000 >= ok_stable_ms:
                break
        else:
            t_ok = None

        if (time.time() - t0) > TIMEOUT_SEC:
            ev_err("TORQUE_TIMEOUT", f"Момент не досягнуто за {TIMEOUT_SEC}s")
            io.set_relay("R04_C2", False); io.set_relay("R06_DI1_POT", False)
            wait_sensor(io, "GER_C2_UP", True, 2.0)
            send_cmd(ser, "WORK")
            return False

        time.sleep(0.005)

    # успех
    io.set_relay("R04_C2", False)
    io.set_relay("R06_DI1_POT", False)
    wait_sensor(io, "GER_C2_UP", True, TIMEOUT_SEC)
    io.pulse("R05_DI4_FREE", ms=FREE_BURST_MS)
    ev_info("TORQUE_OK", "Момент досягнуто, інструмент піднято", stable_ms=ok_stable_ms)
    return True


# =====================[ ГЛАВНАЯ ЛОГИКА ]=======================
def main():
    # 0) выбрать устройство из конфигурации (требуем аргумент)
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True, help="device key from devices.yaml")
    args = parser.parse_args()

    ev_info("BOOT", "Скрипт циклу запущено")

    devices = load_devices_config()
    dev = devices.get(args.device)
    if not dev:
        print(f"[err] device '{args.device}' не знайдено в devices.yaml")
        raise SystemExit(1)
    program = dev["program"]
    print(f"[info] Пристрій: {dev['name']} (отвори={dev.get('holes')})")

    # 1) инициализация GPIO и триггера
    io = IOController()
    trg = StartTrigger(TRIGGER_HOST, TRIGGER_PORT)
    trg.start()


    ui_status_update(status_text="Перевіряю наявність сжатого повітря", can_tighten=False, phase="check_pneumatics")
    # 1.1 Перевірка стартової пневматики (UP=CLOSE, DOWN=OPEN) — inline
    up   = io.sensor_state("GER_C2_UP")      # True = CLOSE
    down = io.sensor_state("GER_C2_DOWN")    # True = CLOSE
    if up is True and down is False:
        ev_info("PNEUM_START_OK", "Стартові стани валідні (UP=CLOSE, DOWN=OPEN)")
    else:
        # 1) статус для UI
        err_msg = "Аварія: скоріш за все не підключене повітря"
        ui_status_update(status_text=err_msg, can_tighten=False, phase="error_no_air")

        # 2) лог + причина завершення
        details = {
            "expect_up": True, "expect_down": False,
            "actual_up": up, "actual_down": down
        }
        ev_err("PNEUMATICS_NOT_READY",
            "Перевірте пневматику: на старті очікуємо GER_C2_UP=CLOSE, GER_C2_DOWN=OPEN.",
            popup=True, **details)
        write_exit_reason("pneumatics_not_ready", err_msg, details)

    # 3) безопасное состояние: момент OFF, цилиндр вверх
        try:
            io.set_relay("R06_DI1_POT", False)
            io.set_relay("R04_C2", False)
        except Exception:
            pass

        # 4) аккуратное завершение
        try:
            trg.stop()
        except Exception:
            pass
        try:
            io.cleanup()
        except Exception:
            pass

        raise SystemExit(2)


    # 1.2 Прогон цилиндра вниз→вверх
    if not cyl_down_up(io):
    # причина уже записана, выходим
        raise SystemExit(2)
    
    ev_info("PNEUM_DU_OK", "Циліндр: вниз/вгору виконано")
    ui_status_update(status_text="Обираю таску для закручування 0", can_tighten=False, phase="select_task0")
    # 1.3 Импульс таски-0 (500 мс)
    select_task0(io, ms=500)
    ev_info("TASK0_SELECT", "Застосовано таск 0", ms=500)

    ui_status_update(status_text="Відкриваю serial-порт", can_tighten=False, phase="open_serial")

# 2) открыть serial и проверить готовность контроллера
    #print(f"[{ts()}] Відкриваю serial-порт {SERIAL_PORT} @ {SERIAL_BAUD}")
    #ser = open_serial()
    #ui_status_update(status_text="Serial-порт выдкрито", can_tighten=False, phase="serial_opened")
    #print(f"[{ts()}] Serial відкрито")



    print(f"[{ts()}] Відкриваю serial-порт {SERIAL_PORT} @ {SERIAL_BAUD}")
    try:
        ser = open_serial()
        if ser is None:
            raise RuntimeError("open_serial() повернула None")

        # успех — статус и лог
        ui_status_update(status_text="Serial-порт відкрито", can_tighten=False, phase="serial_opened")
        ev_info("SER_OPENED", f"Serial відкрито: {SERIAL_PORT} @ {SERIAL_BAUD}")
        print(f"[{ts()}] Serial відкрито")

    except Exception as e:
        # неуспех — статус для UI + лог + причина завершення
        err_msg = f"Аварія: не вдалося відкрити serial ({SERIAL_PORT} @ {SERIAL_BAUD})"
        ev_err("SERIAL_OPEN_FAIL", f"{err_msg}. Помилка: {e}", popup=True)

        ui_status_update(
            status_text=err_msg,
            can_tighten=False,
            phase="error_serial_open"
        )

        write_exit_reason(
            "serial_open_fail",
            err_msg,
            {"port": SERIAL_PORT, "baud": SERIAL_BAUD, "error": str(e)}
        )

        # аккуратне згортання (якщо вже є ініціалізовані об’єкти)
        try:
            trg.stop()
        except Exception:
            pass
        try:
            io.cleanup()
        except Exception:
            pass

        raise SystemExit(2)


    #time.sleep(1.0)

    # СНАЧАЛА почистим входной буфер (если там был мусор/хвост от ресета)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    
    # ЖДЁМ БАННЕР ТОЛЬКО ОДИН РАЗ
    #if not wait_ready(ser, timeout=5.0):
    #    ev_err("READY_TIMEOUT", "Не дочекалися 'ok READY' від контролера")
    #    trg.stop()
     #   io.cleanup()
     #   try: ser.close()
      #  except Exception: pass
      #  raise SystemExit(2)
    #time.sleep(1.0)   # ← твой пункт 1.6 «ждем 1 сек»

    #try:
    #    ser.reset_input_buffer()
    #except Exception:
    #    pass

    # 1.6–1.7 Ждём MOT_X_OK/MOT_Y_OK/ok READY (с авто-RESET при *_ALARM)
   # после: ser = open_serial(); print("Serial відкрито")
    #time.sleep(1.0)
    ui_status_update(status_text="Перевіряю мотори", can_tighten=False, phase="check_motors")

    # === inline: wait_motors_ok_and_ready(ser, timeout=15.0) ===
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    t_end = time.time() + 15.0
    got_x_ok = False
    got_y_ok = False
    got_ready = False
    x_alarm_seen = False
    y_alarm_seen = False

    def _motors_fail_exit(msg: str):
        # статусы/логи как при обычном фейле
        ev_err("READY_TIMEOUT", msg, popup=True)
        ui_status_update(status_text="Аварія: мотори не готові", can_tighten=False, phase="error_motors")
        try: trg.stop()
        except Exception: pass
        try: io.cleanup()
        except Exception: pass
        try: ser.close()
        except Exception: pass
        raise SystemExit(2)

    while time.time() < t_end:
        # быстрый успех
        if got_x_ok and got_y_ok and got_ready:
            print("=== OK: X_OK + Y_OK + ok READY отримані ===")
            break

        raw = ser.readline()
        if not raw:
            continue
        s = raw.decode(errors="ignore").strip()
        if not s:
            continue
        print(f"[SER] {s}")
        ls = s.lower().strip()

        # --- ALARM → RESET ---
        if "mot_x_alarm" in ls:
            x_alarm_seen = True
            ev_info("MOT_X_ALARM", "ALARM на осі X — виконуємо скидання")
            ui_status_update(status_text="ALARM на осі X — виконуємо скидання", can_tighten=False, phase="mot_x_alarm")

            try:
                ser.write(b"MOT_X_RESET\n"); ser.flush()
            except Exception as e:
                ev_err("MOT_X_RESET_FAIL", f"Помилка відправки MOT_X_RESET: {e}", popup=True)
                ui_status_update(status_text="Помилка відправки MOT_X_RESET", can_tighten=False, phase="error_mot_x_reset")
                _motors_fail_exit("Помилка під час скидання ALARM на осі X")
            continue

        if "mot_y_alarm" in ls:
            y_alarm_seen = True
            ev_info("MOT_Y_ALARM", "ALARM на осі Y — виконуємо скидання")
            ui_status_update(status_text="ALARM на осі Y — виконуємо скидання", can_tighten=False, phase="mot_y_alarm")
            try:
                ser.write(b"MOT_Y_RESET\n"); ser.flush()
            except Exception as e:
                ev_err("MOT_Y_RESET_FAIL", f"Помилка відправки MOT_Y_RESET: {e}", popup=True)
                ui_status_update(status_text="Помилка відправки MOT_Y_RESET", can_tighten=False, phase="error_mot_y_reset")
                _motors_fail_exit("Помилка під час скидання ALARM на осі Y")
            continue

        # --- OK маркеры ---
        if "mot_x_ok" in ls:
            if not got_x_ok:
                got_x_ok = True
                ev_info("MOT_X_OK", "Мотор X готовий")
                ui_status_update(status_text="Мотор X готовий", can_tighten=False, phase="mot_x_ok")
            if got_x_ok and got_y_ok and got_ready:
                print("=== OK: X_OK + Y_OK + ok READY отримані ===")
                break
            continue

        if "mot_y_ok" in ls:
            if not got_y_ok:
                got_y_ok = True
                ev_info("MOT_Y_OK", "Мотор Y готовий")
                ui_status_update(status_text="Мотор Y готовий", can_tighten=False, phase="mot_y_ok")
            if got_x_ok and got_y_ok and got_ready:
                print("=== OK: X_OK + Y_OK + ok READY отримані ===")
                break
            continue

        if ls == "ok ready":
            if not got_ready:
                got_ready = True
                ev_info("READY_OK", "Контролер готовий (ok READY)")
                ui_status_update(status_text="Контроллер готовий (ok READY)", can_tighten=False, phase="ready_ok")

            if got_x_ok and got_y_ok and got_ready:
                print("=== OK: X_OK + Y_OK + ok READY отримані ===")
                break
            continue

        if ls.startswith("err"):
            ev_err("SER_ERR", f"Контролер відповів помилкою: {s}", popup=True)
            _motors_fail_exit("Помилка від контролера під час перевірки моторів")
            ui_status_update(status_text="Помилка від контролера під час перевірки моторів", can_tighten=False, phase="error_controller")

    # если цикл закончился без break → таймаут
    if not (got_x_ok and got_y_ok and got_ready):
        _motors_fail_exit(
            f"Не отримали всі маркери за 15.0s "
            f"(X_OK={got_x_ok}, Y_OK={got_y_ok}, READY={got_ready}; "
            f"X_ALARM_SEEN={x_alarm_seen}, Y_ALARM_SEEN={y_alarm_seen})"
        )
        ui_status_update(status_text="Не отримали всі маркери за 15.0sв", can_tighten=False, phase="error_motors_timeout")
# === /inline ===



    #time.sleep(5.0)
    time.sleep(2.0)
    # 3) базовая инициализация координатной системы
    # 1.6 Хоуминг G28 и ожидание IN_HOME_POS / ok
    #if not home_to_zero(ser, timeout=10.0):
        # причина уже записана, выходим; закрытие — в finally
    #    raise SystemExit(2)
    #print("=== В нулях ===")
    # 1.7 Переход у WORK (ждём 'ok')
    #if not go_work(ser, timeout=10.0):
    #    raise SystemExit(2)
    
    print("=== Старт скрипта ===")
    ui_status_update(status_text="Виходжу в нулі...", can_tighten=False, phase="homing")
    send_cmd(ser, "G28")   # хоуминг
    ui_status_update(status_text="Виходжу до оператора...", can_tighten=False, phase="to_operator")
    send_cmd(ser, "WORK")  # привести механику в безопасный «рабочий» пресет
    ev_info("WORK", "Систему переведено у робочий пресет")
    


    # локальный хелпер перемещения (если у тебя есть глобальный move_xy — можешь использовать его)
    def _move_xy(ser_, x, y, f=None, cmd="G"):
        feed = f or MOVE_F
        send_cmd(ser_, f"{cmd} X{x} Y{y} F{feed}")

    try:
        while True:
            # ——— ожидание запуска ———
            set_cycle_busy(False)
            ui_status_update(
            status_text="Готово - покладіть девайс і натисніть старт",
            can_tighten=True,
            phase="ready"
            )
            print("[cycle] Чекаю педаль/START ...")
            if not wait_pedal_or_command(io, trg):
                ev_info("ABORT_IDLE", "Старт не отримано — виходжу")
                break

            ev_info("CYCLE_START", "Старт циклу (педаль/команда)")
            ui_status_update(status_text="Виконую цикл закручування, чекайте…", can_tighten=False, phase="running")
            set_cycle_busy(True)

            # ——— исполнение маршрута из devices.yaml ———
            abort = False
            area_armed = False  # контроль шторы включим после 1-й точки

            for idx, step in enumerate(program):
                t = step.get("type", "free")
                x = step["x"]; y = step["y"]; f = step.get("f")
                feed_val = int(f or MOVE_F)

                # перемещение в точку шага
                ev_info("MOVE", f"Перехід X={x} Y={y}", x=float(x), y=float(y), f=feed_val)
                # выбор команды: для free → GF, для work → G
                cmd = "GF" if t == "free" else "G"
                _move_xy(ser, x, y, f, cmd=cmd)

                # включаем контроль шторы после первой достигнутой точки
                if not area_armed and idx == 0:
                    area_armed = True
                    ev_info("AREA_ARM", "Контроль штори активовано (з точки 1 і до кінця серії)")

                # быстрый safety-чек сразу после перемещения
                if area_armed and io.sensor_state("AREA_SENSOR"):
                    ev_alarm("AREA_TRIP", "Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.")
                    ui_status_update(status_text="Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.", can_tighten=False, phase="area_trip")
                    io.set_relay("R06_DI1_POT", False)
                    io.set_relay("R04_C2", False)
                    try: wait_sensor(io, "GER_C2_UP", True, 2.0)
                    except Exception: pass
                    send_cmd(ser, "WORK")
                    abort = True
                    break

                if t == "work":
                    ok_pos, meta = verify_position_reached(ser, x, y, tol_mm=0.30)
                    if not ok_pos:
                        ev_warn("POS_MISMATCH", "Позиція не в межах допуску після руху", target_x=float(x), target_y=float(y), **meta)
                        slow_f = max(2000, int(feed_val // 3))
                        _move_xy(ser, x, y, slow_f, cmd="G")
                        time.sleep(0.05)
                        ok_pos2, meta2 = verify_position_reached(ser, x, y, tol_mm=0.30)
                        if not ok_pos2:
                            ev_err("POS_VERIFY_FAIL", "Не вдалося підтвердити позицію. Аварійний вихід перед закручуванням", target_x=float(x), target_y=float(y), **meta2)
                            ui_status_update(status_text="Помилка позиціювання — відміна кроку", can_tighten=False, phase="pos_fail")
                            send_cmd(ser, "WORK")
                            abort = True
                            break
                    # (необ.) импульс выбора таски
                    if "task" in step:
                        select_task(io, int(step["task"]))
                        ev_info("TASK_SELECT", f"Выбрана таска {int(step['task'])}", task=int(step["task"]))

                    # перед подачей — ещё раз safety
                    if area_armed and io.sensor_state("AREA_SENSOR"):
                        ev_alarm("AREA_TRIP", "Спрацював захисний сенсор перед подачею. Зупинка. РУКИ ГЕТЬ.")
                        ui_status_update(status_text="Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.", can_tighten=False, phase="area_trip")
                        io.set_relay("R06_DI1_POT", False)
                        io.set_relay("R04_C2", False)
                        try: wait_sensor(io, "GER_C2_UP", True, 2.0)
                        except Exception: pass
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                    # подача винта до импульса IND_SCRW
                    if not feed_until_detect(io):
                        ev_alarm("FEED_FAIL", "Подача гвинта неуспішна - аварійний вихід")
                        ui_status_update(status_text="Подача гвинта неуспішна - аварійний вихід", can_tighten=False, phase="feed_fail")
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                    # перед закруткой — снова safety
                    if area_armed and io.sensor_state("AREA_SENSOR"):
                        ev_alarm("AREA_TRIP", "Спрацював захисний сенсор перед закручуванням. Зупинка. РУКИ ГЕТЬ.")
                        ui_status_update(status_text="Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.", can_tighten=False, phase="area_trip")
                        io.set_relay("R06_DI1_POT", False)
                        io.set_relay("R04_C2", False)
                        try: wait_sensor(io, "GER_C2_UP", True, 2.0)
                        except Exception: pass
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                    # закрутка по моменту (с поддержкой контроля шторы внутри, если есть)
                    if "torque_sequence_with_area" in globals():
                        ok_torque = torque_sequence_with_area(io, ser, area_armed)
                    else:
                        ok_torque = torque_sequence(io)

                    if not ok_torque:
                        ev_err("TORQUE_FAIL", "Момент не досягнуто/аварія під час процесу — Зупинка. РУКИ ГЕТЬ.")
                        ui_status_update(status_text="Спрацював захисний сенсор. Зупинка. РУКИ ГЕТЬ.", can_tighten=False, phase="area_trip")
                        send_cmd(ser, "WORK")
                        abort = True
                        break

                # небольшая тех.пауза (сгладить дребезг статусов)
                time.sleep(0.05)

            # ——— окончание серии ———
            send_cmd(ser, "WORK")
            if area_armed:
                ev_info("AREA_DISARM", "Контроль шторы отключён (конец серии)")

            if abort:
                ev_info("CYCLE_ABORT", "Серия прервана аварией/ошибкой")
            else:
                ev_info("CYCLE_DONE", "Серію кроків завершено")

            set_cycle_busy(False)

            # цикл продолжается: снова ждём педаль/команду
            # если нужен одноразовый запуск — раскомментируй:
            # break

    except KeyboardInterrupt:
        print("[cycle] Перервано користувачем (Ctrl+C)")
        ev_info("CYCLE", "Перервано користувачем")
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
        print("=== Зупинено. GPIO звільнено, serial закрито ===")
        ev_info("SHUTDOWN", "Зупинено, GPIO/serial закриті")

if __name__ == "__main__":
    main()