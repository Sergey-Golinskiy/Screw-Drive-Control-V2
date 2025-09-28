#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
# если нет переменных DISPLAY/WAYLAND_DISPLAY — поднимем eglfs
if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    # работаем через EGLFS — жёстко фиксируем масштаб
    os.environ.setdefault("QT_QPA_PLATFORM", "eglfs")
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "0")
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "0")
    os.environ.setdefault("QT_SCALE_FACTOR", "1")
    os.environ.setdefault("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough")
import socket
import os, sys, socket, re, time
import requests
import json
from pathlib import Path
from functools import partial
import html

from PyQt5.QtCore import Qt, QTimer, QThread, QEvent, QCoreApplication, pyqtSignal as Signal
QCoreApplication.setAttribute(Qt.AA_DisableHighDpiScaling, True)
from PyQt5.QtGui import QFont # type: ignore
from PyQt5.QtGui import QPixmap # type: ignore
from PyQt5.QtWidgets import ( # type: ignore
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QTabWidget, QLabel, QPushButton, QFrame, QComboBox, QLineEdit,
    QTextEdit, QSpinBox, QSizePolicy, QScrollArea
)

# --- GPIO (Raspberry Pi) ---
try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None

# Параметры «педали»
PEDAL_GPIO_PIN = 18        # BCM 18 (физический пин 12)
PEDAL_ACTIVE_LOW = True    # если педаль замыкается на «землю» — оставь True
PEDAL_PULSE_MS = 120       # длительность импульса
EVENTS_LOG_PATH = Path("/tmp/screw_events.jsonl")
# ================== Конфиг ==================
API_BASE = os.getenv("API_BASE", "http://127.0.0.1:8000/api")
POLL_MS   = 1000
BORDER_W  = 10

# === NEW: единый источник статуса для UI ===
UI_STATUS_PATH = Path("/tmp/ui_status.json")

def send_start_trigger(host="127.0.0.1", port=8765, payload=b"START\n", timeout=0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall(payload)
            s.settimeout(timeout)
            resp = s.recv(64)
            return b"OK" in resp.upper()
    except Exception:
        return False

def is_port_open(host="127.0.0.1", port=8765, timeout=0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

def send_start_trigger_with_retry(host="127.0.0.1", port=8765, payload=b"START\n",
                                  timeout=0.5, retries=15, delay=0.2) -> bool:
    for _ in range(retries):
        try:
            with socket.create_connection((host, port), timeout=timeout) as s:
                s.sendall(payload)
                s.settimeout(timeout)
                resp = s.recv(64)
                if b"OK" in resp.upper():
                    return True
        except Exception:
            pass
        time.sleep(delay)
    return False

# ================== HTTP ==================
def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "Unknown"

def req_get(path: str):
    url = f"{API_BASE}/{path.lstrip('/')}"
    r = requests.get(url, timeout=3)
    r.raise_for_status()
    return r.json()

def req_post(path: str, payload=None):
    url = f"{API_BASE}/{path.lstrip('/')}"
    r = requests.post(url, json=payload or {}, timeout=5)
    r.raise_for_status()
    return r.json()

_gpio_initialized = False

def gpio_pedal_init():
    """Инициализация GPIO для педали (выход)."""
    global _gpio_initialized
    if GPIO is None or _gpio_initialized:
        return
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    inactive = GPIO.HIGH if PEDAL_ACTIVE_LOW else GPIO.LOW
    GPIO.setup(PEDAL_GPIO_PIN, GPIO.OUT, initial=inactive)
    _gpio_initialized = True

def gpio_pedal_pulse(ms: int = PEDAL_PULSE_MS):
    if GPIO is None:
        raise RuntimeError("RPi.GPIO is not installed. Install: sudo apt install -y python3-rpi.gpio")
    if not _gpio_initialized:
        gpio_pedal_init()
    active = GPIO.LOW if PEDAL_ACTIVE_LOW else GPIO.HIGH
    inactive = GPIO.HIGH if PEDAL_ACTIVE_LOW else GPIO.LOW
    GPIO.output(PEDAL_GPIO_PIN, active)
    time.sleep(ms / 1000.0)
    GPIO.output(PEDAL_GPIO_PIN, inactive)

def gpio_cleanup():
    if GPIO is not None:
        try: GPIO.cleanup()
        except Exception: pass


class ApiClient:
    def status(self):           return req_get("status")
    def ext_start(self):        return req_post("ext/start")
    def ext_stop(self):         return req_post("ext/stop")
    def relay(self, name, action, ms=None):
        data = {"name": name, "action": action}
        if action == "pulse" and ms:
            data["ms"] = int(ms)
        return req_post("relay", data)

    # --- NEW: pedal emulation (safe fallbacks) ---
    def pedal(self, relay_name="PEDAL", pulse_ms=120):
        try:
            # если сервер поддерживает прямой эндпоинт
            return req_post("pedal", {"ms": pulse_ms})
        except Exception:
            # Fallback: реле PEDAL импульсом
            return self.relay(relay_name, "pulse", pulse_ms)

    # --- NEW: stop script (optional endpoint; fallback to ext_stop) ---
    def script_stop(self):
        try:
            return req_post("script/stop", {})
        except Exception:
            return self.ext_stop()
    
    def config(self) -> dict:
        """GET /api/config -> {"devices":[{key,name,holes}], "selected": "<key>|None"}"""
        return req_get("config")

    def select(self, key: str) -> dict:
        """POST /api/select {"key": key} -> {"ok":true, ...}"""
        return req_post("select", {"key": key})


# ================== Serial ==================
try:
    import serial, serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None

class SerialReader(QThread):
    line   = Signal(str)
    opened = Signal(bool)

    def __init__(self):
        super().__init__()
        self._ser  = None
        self._stop = False

    def open(self, port: str, baud: int):
        if serial is None:
            self.line.emit("pyserial is not installed")
            self.opened.emit(False)
            return False
        self.close()
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, rtscts=False, dsrdtr=False)
            # Явно опускаем линии, как ты просил раньше
            try:
                self._ser.dtr = False
                self._ser.rts = False
            except Exception:
                pass
            self._stop = False
            if not self.isRunning():
                self.start()
            self.opened.emit(True)
            self.line.emit(f"[OPEN] {port} @ {baud}")
            return True
        except Exception as e:
            self._ser = None
            self.opened.emit(False)
            self.line.emit(f"[ERROR] open {port}: {e}")
            return False

    def close(self):
        if self._ser:
            try: self._ser.close()
            except Exception: pass
        self._ser = None
        self.opened.emit(False)

    def write(self, text: str):
        if self._ser:
            try:
                if not text.endswith("\n"): text += "\n"
                self._ser.write(text.encode("utf-8"))
            except Exception as e:
                self.line.emit(f"[ERROR] write: {e}")

    def run(self):
        while not self._stop:
            if self._ser:
                try:
                    data = self._ser.readline()
                    if data:
                        try:  s = data.decode("utf-8", "ignore").rstrip()
                        except Exception: s = repr(data)
                        self.line.emit(s)
                except Exception as e:
                    self.line.emit(f"[ERROR] read: {e}")
                    time.sleep(0.2)
            else:
                time.sleep(0.1)

    def stop(self):
        self._stop = True
        self.wait(1000)
        self.close()

# ================== UI helpers ==================
def make_card(title: str) -> QFrame:
    box = QFrame(); box.setObjectName("card")
    lay = QVBoxLayout(box); lay.setContentsMargins(16, 16, 16, 16); lay.setSpacing(10)
    t = QLabel(title); t.setObjectName("cardTitle")
    lay.addWidget(t)
    return box

def big_button(text: str) -> QPushButton:
    b = QPushButton(text)
    b.setObjectName("bigButton")
    b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    b.setMinimumHeight(160)
    b.setCheckable(False)
    return b

# ================== Tabs ==================
class WorkTab(QWidget):
    def __init__(self, api: ApiClient, parent=None):
        super().__init__(parent)
        self.api = api

        root = QVBoxLayout(self); root.setContentsMargins(24,24,24,24); root.setSpacing(18)

        self.ipLabel = QLabel(f"IP: {get_local_ip()}"); self.ipLabel.setObjectName("muted")
        root.addWidget(self.ipLabel, 0, Qt.AlignLeft)

        row = QHBoxLayout(); row.setSpacing(18)
        self.btnPedal = big_button("ЗАКРУТИТИ ГВИНТИ")
        self.btnKill  = big_button("ЗУПИНИТИ")
        # --- Лок-блокировка «один клик за цикл» ---
        self._pedal_locked = False     # True = повторный клик запрещён
        self._pedal_lock_t0 = 0.0      # когда залочили (секунды time.time())
        self._pedal_lock_timeout = 120 # страховочный авто-анлок через N секунд
        # Блокируем педаль до «готовности»
        self.btnPedal.setEnabled(False)
        self.lblPedalStatus = QLabel("Поле статусу...")
        self.lblPedalStatus.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # Лейбл статуса под кнопкой (в том же столбце)
        # Статус-бейдж под кнопкой: яркий и читаемый
        self.lblPedalStatus.setObjectName("statusBadge")
        self.lblPedalStatus.setAlignment(Qt.AlignCenter)
        self.lblPedalStatus.setWordWrap(True)
        #self.lblPedalStatus.setText("Выхожу в нули…")

        # Базовый стиль + варианты (цвет/фон) — можно править под бренд
        self.lblPedalStatus.setStyleSheet("""
QLabel#statusBadge {
    padding: 12px 16px;
    border: 2px solid #5B672D;                 /* тёмно-оливковый из вашей палитры */
    border-radius: 12px;
    background-color: rgba(97,130,52,0.85);    /* тёмнее фон для контраста */
    color: #ffffff;                             /* белый текст */
    font: 600 12pt "Montserrat Light Alt1";    /* жирнее и чуть крупнее */
    letter-spacing: 0.2px;
}

/* Фазовые варианты: подбираем тёмные, чтобы белый текст оставался читабельным */
QLabel#statusBadge[variant="info"] {
    border-color: #5B672D;
    background-color: rgba(73, 94, 32, 0.90);
}
QLabel#statusBadge[variant="success"] {
    border-color: #679E3C;
    background-color: rgba(66, 122, 28, 0.90);
}
QLabel#statusBadge[variant="warning"] {
    border-color: #564C26;
    background-color: rgba(156, 120, 15, 0.92);
}
QLabel#statusBadge[variant="danger"] {
    border-color: #8E2F26;
    background-color: rgba(142, 47, 38, 0.92);
}
        """)


        self.btnKill.setObjectName("stopButton")
        row.addWidget(self.btnPedal); row.addWidget(self.btnKill)
        root.addLayout(row, 1)
        root.addWidget(self.lblPedalStatus)
        root.setSpacing(10)

        #self.lblPedalStatus = QLabel("Status: unknown"); self.lblPedalStatus.setObjectName("state")
        #root.addWidget(self.lblPedalStatus, 0, Qt.AlignLeft)

        self.btnPedal.clicked.connect(self.on_pedal)
        self.btnKill.clicked.connect(self.on_kill)

    def on_pedal(self):
        try:
            # Не даём эмулировать педаль до «готовности»
            if not self.btnPedal.isEnabled():
                self.lblPedalStatus.setText("Ще не готовий (наведення/позиціонування).")
                return
            # Проверим, запущен ли внешний процесс — чтобы не жать в пустоту
            running = False
            try:
                st0 = self.api.status()
                running = bool(st0.get("external_running"))
            except Exception:
                pass

            if not running:
                self.lblPedalStatus.setText(
                'Скрипт не працює. Спочатку натисніть «Запустити програму» на вкладці «Пуск».'
                )
                return

            # Убедимся, что listener поднят (порт 8765). Если нет — пару секунд подождём.
            t0 = time.time()
            while not is_port_open() and time.time() - t0 < 3.0:
                QApplication.processEvents()
                time.sleep(0.1)

            ok = send_start_trigger_with_retry()
            if ok:
                self.lblPedalStatus.setText("Команда START надіслана (емуляція педалей).")
                self.btnPedal.setEnabled(False)         # мгновенно «гасим» кнопку
                self._pedal_locked = True               # <— ставим лок
                self._pedal_lock_t0 = time.time()
            else:
                self.lblPedalStatus.setText("Не вдалося надіслати START (немає відповіді від циклу).")

            # Обновим статус для подсветки кнопок
            try:
                st = self.api.status()
                self.render(st)
            except Exception:
                pass

        except Exception as e:
            self.lblPedalStatus.setText(f"Помилка надсилання START: {e}")




    def on_kill(self):
        try:
            # если есть api/script/stop — добавь в ApiClient; здесь на всякий случай остановим external
            st = self.api.ext_stop()
            self.render(st)
        except Exception as e:
            self.lblPedalStatus.setText(f"Помилка зупинки: {e}")

    def _read_ui_status_file(self) -> dict:
    
        try:
            if UI_STATUS_PATH.exists():
                return json.loads(UI_STATUS_PATH.read_text())
        except Exception:
            pass
    # дефолт до готовности
        return {"status_text": "Статус", "can_tighten": False, "phase": "homing"}

    def _apply_ui_status(self, data: dict):
        self.btnPedal.setEnabled(bool(data.get("can_tighten", False)))
        self.lblPedalStatus.setText(data.get("status_text", "") or "")


    def render(self, st: dict):
        running = bool(st.get("external_running"))
        #running = bool(st.get("external_running"))
        #self.lblPedalStatus.setText("Status: " + ("PROGRAM RUNNING" if running else "PROGRAM STOPPED"))
        # === NEW: применим статус «готовности/хоминга» из /tmp/ui_status.json ===
    
        try:
            ui = self._read_ui_status_file()
        except Exception:
            ui = {"status_text": "Немає зв'язку зі статусом.", "can_tighten": False}
        # Текст и доступность кнопки
        self.lblPedalStatus.setText(ui.get("status_text", "") or "")
        self.btnPedal.setEnabled(bool(ui.get("can_tighten", False)))

        # Подсветка по фазе (homing/ready/running/alarm)
        phase = (ui.get("phase") or "").lower()
        variant = {
            "homing":  "warning",
            "ready":   "success",
            "running": "info",
            "alarm":   "danger",
        }.get(phase, "info")

        # Применяем вариант и «переполировываем» виджет, чтобы CSS селектор сработал
        self.lblPedalStatus.setProperty("variant", variant)
        self.lblPedalStatus.style().unpolish(self.lblPedalStatus)
        self.lblPedalStatus.style().polish(self.lblPedalStatus)



        # === НОВОЕ: подсветка «Эмуляции педали», пока цикл ЗАНЯТ между нажатиями ===
        busy = bool(st.get("cycle_busy"))
        # когда busy=True — делаем кнопку зелёной
        self.btnPedal.setProperty("ok", busy)

        if self._pedal_locked and can and (phase in ("ready", "")) and not busy:
            self._pedal_locked = False

        # актуальность «Стоп скрипта» как раньше
        self.btnKill.setProperty("ok", running)

        for w in (self.btnPedal, self.btnKill):
            w.style().unpolish(w); w.style().polish(w)



#from PyQt5.QtWidgets import QDialog, QDialogButtonBox, QVBoxLayout, QLabel # type: ignore


class ServiceTab(QWidget):
    def __init__(self, api: ApiClient, parent=None):
        super().__init__(parent)
        self.api = api
        self._relay_widgets = {}  # name -> (lblState, spin, btnOn, btnOff, btnPulse)

        root = QHBoxLayout(self); root.setContentsMargins(24,24,24,24); root.setSpacing(18)

        # Левая колонка
        left = QVBoxLayout(); left.setSpacing(18)

        self.sensorsCard = make_card("Сенсори / кінцевики")
        self.sensorsGrid = QGridLayout(); self.sensorsGrid.setHorizontalSpacing(14); self.sensorsGrid.setVerticalSpacing(10)
        self.sensorsCard.layout().addLayout(self.sensorsGrid)
        left.addWidget(self.sensorsCard)

        self.relaysCard = make_card("Реле (ON/OFF/PULSE)")
        self.relaysGrid = QGridLayout(); self.relaysGrid.setHorizontalSpacing(8); self.relaysGrid.setVerticalSpacing(8)
        self.relaysCard.layout().addLayout(self.relaysGrid)
        left.addWidget(self.relaysCard, 1)
        
        # Правая колонка — Serial
        right = QVBoxLayout(); right.setSpacing(18)
        
        self.setStyleSheet("""
        ServiceTab QLabel { color: #ffffff; }
        """)
        

        # ---- Network / IP card ----
        self.netCard = make_card("Мережа")
        net = self.netCard.layout()
        row = QHBoxLayout()
        self.lblIp = QLabel("IP: —")
        self.btnIpRefresh = QPushButton("Оновити")
        row.addWidget(self.lblIp, 1)
        row.addWidget(self.btnIpRefresh)
        net.addLayout(row)
        right.addWidget(self.netCard)
        self.lblIp.setStyleSheet("color: #eef3ff; font-weight: 600;")

        # таймер авто-обновления IP
        self._ipTimer = QTimer(self)
        self._ipTimer.setInterval(3000)      # каждые 3 секунды
        self._ipTimer.timeout.connect(self._refresh_ip)
        self._ipTimer.start()

        # кнопка ручного обновления
        self.btnIpRefresh.clicked.connect(self._refresh_ip)

        # первичное заполнение
        self._refresh_ip()


        
        self.serialCard = make_card("Arduino Serial")
        sc = self.serialCard.layout()

        top = QHBoxLayout()
        self.cbPort = QComboBox(); self.cbBaud = QComboBox()
        for b in (9600, 115200, 230400): self.cbBaud.addItem(str(b))
        self.btnRefresh = QPushButton("Refresh")
        self.btnOpen = QPushButton("Open")
        self.btnClose = QPushButton("Close")
        top.addWidget(QLabel("Port:")); top.addWidget(self.cbPort, 1)
        top.addWidget(QLabel("Speed:")); top.addWidget(self.cbBaud)
        top.addWidget(self.btnRefresh); top.addWidget(self.btnOpen); top.addWidget(self.btnClose)
        sc.addLayout(top)

        self.txtLog = QTextEdit(); self.txtLog.setReadOnly(True); self.txtLog.setMinimumHeight(240)
        sc.addWidget(self.txtLog, 1)

        send = QHBoxLayout()
        self.edSend = QLineEdit(); self.edSend.setPlaceholderText("Enter the command and press Send (\\n will be added)")
        self.btnSend = QPushButton("Send")
        send.addWidget(self.edSend, 1); send.addWidget(self.btnSend)
        sc.addLayout(send)

        #self.vkeyboard = VirtualKeyboard(self)
        #self.vkeyboard.on_enter = self.send_serial
        #self.edSend.installEventFilter(self)
        right.addWidget(self.serialCard, 1)



        root.addLayout(left, 2)
        root.addLayout(right, 1)

        # Serial backend
        self.reader = SerialReader()
        self.reader.line.connect(self.log_line)
        self.reader.opened.connect(self.serial_opened)
        self.btnRefresh.clicked.connect(self.fill_ports)
        self.btnOpen.clicked.connect(self.open_serial)
        self.btnClose.clicked.connect(self.reader.close)
        self.btnSend.clicked.connect(self.send_serial)

        self.fill_ports()

    # --- реле ---
    def _relay_cell(self, row: int, name: str):
        lblName = QLabel(name); lblName.setObjectName("badge")
        lblState = QLabel("—"); lblState.setObjectName("stateOnOff")
        spin = QSpinBox(); spin.setRange(20, 5000); spin.setValue(150); spin.setSuffix(" ms"); spin.setFixedWidth(110)
        btnOn  = QPushButton("ON"); btnOff = QPushButton("OFF"); btnPulse = QPushButton("PULSE")
        btnOn.clicked.connect(partial(self._relay_cmd, name, "on"))
        btnOff.clicked.connect(partial(self._relay_cmd, name, "off"))
        btnPulse.clicked.connect(lambda: self._relay_cmd(name, "pulse", spin.value()))
        self.relaysGrid.addWidget(lblName,  row, 0)
        self.relaysGrid.addWidget(lblState, row, 1)
        self.relaysGrid.addWidget(btnOn,    row, 2)
        self.relaysGrid.addWidget(btnOff,   row, 3)
        self.relaysGrid.addWidget(spin,     row, 4)
        self.relaysGrid.addWidget(btnPulse, row, 5)
        self._relay_widgets[name] = (lblState, spin, btnOn, btnOff, btnPulse)

    def _relay_cmd(self, name: str, action: str, ms: int | None = None):
        try:
            data = self.api.relay(name, action, ms)
            self.render(data)
        except requests.HTTPError as e:
            self.log_line(f"[HTTP {e.response.status_code}] Ручне керування недоступне КОЛИ ПРОГРАМА ПРАЦЮЄ")
        except Exception as e:
            self.log_line(f"[ERROR] relay: {e}")
   
    def _get_ip_address(self) -> str:
        """
        Возвращает 'красивый' текущий IP хоста.
        1) пытаемся через 'hostname -I' (даёт все адреса без лишнего);
        2) если пусто — делаем UDP-‘подключение’ к 8.8.8.8:80 и берём локальный IP сокета;
        3) если совсем плохо — 127.0.0.1.
        """
        try:
            import subprocess, shlex
            out = subprocess.check_output(shlex.split("hostname -I"), timeout=0.5).decode("utf-8", "ignore").strip()
            if out:
                # берём первый непетлевой адрес
                parts = [p for p in out.split() if not p.startswith("127.")]
                if parts:
                    return parts[0]
        except Exception:
            pass
        # fallback через UDP ‘connect’
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.2)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip or "127.0.0.1"
        except Exception:
            return "127.0.0.1"

    def _refresh_ip(self):
        ip = self._get_ip_address()
        self.lblIp.setText(f"IP: {ip}")


    # --- serial ---
    def fill_ports(self):
        self.cbPort.clear()
        ports = []
        if list_ports:
            try:
                ports = [p.device for p in list_ports.comports()]
            except Exception:
                ports = []
        for p in ["/dev/ttyACM0", "/dev/ttyUSB0"]:
            if p not in ports: ports.append(p)
        for p in ports: self.cbPort.addItem(p)

    def open_serial(self):
        port = self.cbPort.currentText().strip()
        baud = int(self.cbBaud.currentText())
        if port: self.reader.open(port, baud)

    def send_serial(self):
        text = self.edSend.text().strip()
        if text:
            self.reader.write(text)
            self.edSend.clear()
            #self.vkeyboard.hide()
            self.edSend.clearFocus()

    def serial_opened(self, ok: bool):
        self.btnOpen.setEnabled(not ok)
        self.btnClose.setEnabled(ok)
        self.btnSend.setEnabled(ok)
        self.cbPort.setEnabled(not ok); self.cbBaud.setEnabled(not ok)

    def log_line(self, s: str):
        self.txtLog.append(s)

    # --- render ---
    def render(self, st: dict):
        # sensors
        names = st.get("sensor_names", [])
        states = st.get("sensors", {})

        # rebuild sensors grid
        while self.sensorsGrid.count():
            item = self.sensorsGrid.takeAt(0)
            w = item.widget()
            if w: w.deleteLater()

        for i, name in enumerate(names):
            lab = QLabel(name); lab.setObjectName("badge")
            v = bool(states.get(name))
            val = QLabel("CLOSE" if v else "OPEN")
            val.setObjectName("ok" if v else "off")
            self.sensorsGrid.addWidget(lab, i, 0)
            self.sensorsGrid.addWidget(val, i, 1)

        # relays
        relay_names = st.get("relay_names", [])
        relays = st.get("relays", {})

        if set(relay_names) != set(self._relay_widgets.keys()):
            while self.relaysGrid.count():
                item = self.relaysGrid.takeAt(0)
                w = item.widget()
                if w: w.deleteLater()
            self._relay_widgets.clear()
            for i, name in enumerate(relay_names):
                self._relay_cell(i, name)

        external = bool(st.get("external_running"))
        for name, widgets in self._relay_widgets.items():
            lblState, spin, btnOn, btnOff, btnPulse = widgets
            is_on = bool(relays.get(name))
            lblState.setText("ON" if is_on else "OFF")
            lblState.setProperty("on", is_on)
            for w in (spin, btnOn, btnOff, btnPulse):
                w.setEnabled(not external)
            lblState.style().unpolish(lblState); lblState.style().polish(lblState)
    


class StartTab(QWidget):
    """Вкладка START: слева список устройств (кнопки), справа вертикально START/STOP."""
    def __init__(self, api: ApiClient, parent=None):
        super().__init__(parent)
        self.api = api
        self.on_started = None  # коллбек задаст MainWindow

        # === корневой лэйаут вкладки: ДВЕ КОЛОНКИ ===
        root = QHBoxLayout(self); 
        root.setContentsMargins(24,24,24,24); 
        root.setSpacing(18)

        # --------- ЛЕВАЯ КОЛОНКА (≈30%) — Список устройств ----------
        left = QVBoxLayout(); left.setSpacing(12)

        self.devCard = make_card("Девайси")
        card_lay = self.devCard.layout()

        # прокручиваемая область с кнопками устройств
        self.devScroll = QScrollArea(); 
        self.devScroll.setWidgetResizable(True)
        self.devList = QWidget()
        self.devListLay = QVBoxLayout(self.devList); 
        self.devListLay.setContentsMargins(0,0,0,0); 
        self.devListLay.setSpacing(10)
        self.devScroll.setWidget(self.devList)

        card_lay.addWidget(self.devScroll)
        left.addWidget(self.devCard)

        # --------- ПРАВАЯ КОЛОНКА (≈70%) — Кнопки START/STOP вертикально ----------
        right = QVBoxLayout(); right.setSpacing(18)




        self.btnStart = big_button("ПОЧАТИ РОБОТУ")
        #self.btnStop  = big_button("STOP program")
        #self.btnStop.setObjectName("stopButton")
        self.txtEvents = QTextEdit()
        self.txtEvents.setReadOnly(True)
        self.txtEvents.setLineWrapMode(QTextEdit.NoWrap)
        self.txtEvents.setFont(QFont("DejaVu Sans Mono", 10))
        self.txtEvents.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.txtEvents.setObjectName("eventsLog")

        # заголовок над логом (по желанию)
        self.lblEventsTitle = QLabel("Останні події")
        self.lblEventsTitle.setStyleSheet("font: 50 6pt 'Montserrat Light Alt1'; margin-top:1px; margin-bottom:1px;color: #FFFFFF;text-transform: capitalize;")

# стиль самого лога
        self.txtEvents.setStyleSheet("""
QTextEdit#eventsLog {
    background: #0f1115;
    color: #FFFFFF;
    border: 1px solid #FFFFFF;
    border-radius: 8px;
    padding: 8px;
    font: 6pt 'Montserrat Light Alt1';
}
        """)

        # добавляем в layout вкладки Start
        

        # делаем кнопки побольше и с одинаковой высотой
        self.btnStart.setMinimumHeight(220)
        self.txtEvents.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        #self.btnStop.setMinimumHeight(220)

        right.addWidget(self.btnStart, 2)          # вес 2
        right.addWidget(self.lblEventsTitle, 0)    # заголовок почти без влияния на растяжение
        right.addWidget(self.txtEvents, 1)         # вес 1 → ≈ половина высоты кнопки



        #right.addWidget(self.btnStart, 1)
        #right.addWidget(self.btnStart, 2)
        #right.addWidget(self.lblEventsTitle, 0)
        #right.addWidget(self.txtEvents, 1)


        #right.addWidget(self.btnStop,  1)

        # подпись состояния
        self.lblPedalStatus = QLabel("Статус: unknown"); 
        self.lblPedalStatus.setObjectName("state")
        right.addWidget(self.lblPedalStatus, 0, Qt.AlignLeft)

        # собрать две колонки в корневой лэйаут (30/70 через стили растяжения)
        root.addLayout(left, 3)
        root.addLayout(right, 7)

        # --- состояние и связи ---
        self._devices = []          # [{"key","name","holes"}, ...]
        self._selected_key = None
        self._cfg_refresh_ts = 0.0
        self._device_buttons = {}   # key -> QPushButton

        self.btnStart.clicked.connect(self.on_start)
        QTimer.singleShot(0, self._resize_events_to_half)
        #self.btnStop.clicked.connect(self.on_stop)

        # первичный рендер (чтоб сразу увидеть устройства, если API доступен)
        try:
            self.render(self.api.status())
        except Exception:
            pass

    def _resize_events_to_half(self):
        """Подгоняет высоту лога примерно под половину высоты кнопки START."""
        h_btn = self.btnStart.height() or self.btnStart.sizeHint().height()
        self.txtEvents.setFixedHeight(max(120, int(h_btn * 0.5)))  # не меньше 120 px для читаемости






    def on_start(self):
        if not self._selected_key:
            self.lblPedalStatus.setText("Спочатку виберіть пристрій.")
            return
        try:
            self.lblPedalStatus.setText("Запуск програми…")
            self.api.ext_start()
            self.lblPedalStatus.setText('Програма запущена. Щоб розпочати роботу, натисніть кнопку «ПУСК» на вкладці «Робота».')
            if callable(self.on_started):
                self.on_started()
        except Exception as e:
            self.lblPedalStatus.setText(f"Не вдалося запустити програму: {e}")

    def _read_log_tail(self, path: Path, max_lines: int = 80) -> list[str]:
        """Быстро читаем последние max_lines строк из JSONL-файла."""
        if not path.exists():
            return ["(лог поки порожній)"]
        # безопасно и без больших затрат по памяти
        lines = []
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                lines.append(line.rstrip("\n"))
                if len(lines) > max_lines * 2:
                    # контролируем рост списка, сдвигая окно
                    lines = lines[-max_lines*2:]
        return lines[-max_lines:] or ["(нет записей)"]

    def _format_event_line(self, line: str) -> str:
        """Парсим JSONL и возвращаем HTML для QTextEdit."""
        import json, datetime
        try:
            evt = json.loads(line)
            # поля времени
            ts = evt.get("ts") or evt.get("time") or ""
            if isinstance(ts, (int, float)):
                ts = datetime.datetime.fromtimestamp(ts).isoformat(timespec="seconds")
            ts = str(ts)

            # уровень/имя/сообщение
            level = (evt.get("level") or evt.get("lvl") or "").upper() or "INFO"
            name  = evt.get("event") or evt.get("type") or evt.get("name") or "event"
            msg   = evt.get("msg") or evt.get("message") or ""

            # экранируем пользовательский текст
            ts_h   = html.escape(ts)
            level_h= html.escape(level)
            name_h = html.escape(str(name))
            msg_h  = html.escape(str(msg))

            # цвета уровней (тёмная тема)
            color = {
                "ERROR": "#ff6b6b",
                "WARN":  "#ffc857",
                "WARNING":"#ffc857",
                "INFO":  "#e6e6e6",
                "DEBUG": "#9fb3c8",
            }.get(level, "#e6e6e6")

            head = f"[{ts_h}] {level_h}"
            body = f"{name_h} — {msg_h}" if msg_h else name_h

            return f'<span style="color:{color}">{head}  {body}</span>'
        except Exception:
            # не-JSON — отдаём как есть, но экранируем
            return f'<span style="color:#e6e6e6">{html.escape(line)}</span>'

    def _refresh_events_log(self):
        lines = self._read_log_tail(EVENTS_LOG_PATH, max_lines=100)
        html_lines = [self._format_event_line(ln) for ln in lines]
        self.txtEvents.setHtml("<br>".join(html_lines))

        # прокрутка в самый низ
        cursor = self.txtEvents.textCursor()
        cursor.movePosition(cursor.End)
        self.txtEvents.setTextCursor(cursor)



        # --- создание/перестройка списка устройств слева ---
    def _rebuild_devices(self, devices: list[dict]):
        # убрать старые кнопки
        for i in reversed(range(self.devListLay.count())):
            w = self.devListLay.itemAt(i).widget()
            if w: w.setParent(None); w.deleteLater()
        self._device_buttons.clear()

        # создать кнопки
        for d in devices:
            key = d.get("key")
            name = d.get("name", key or "?")
            holes = d.get("holes")
            text = f"{name}" + (f"\n{holes} holes" if holes is not None else "")

            btn = QPushButton(text)
            btn.setObjectName("devButton")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.setMinimumHeight(80)
            btn.setCheckable(False)
            btn.clicked.connect(lambda _=False, k=key: self._select_device(k))

            self.devListLay.addWidget(btn)
            self._device_buttons[key] = btn

        self.devListLay.addStretch(1)
        self._apply_dev_styles()

    def _apply_dev_styles(self):
        # подсветка выбранного девайса
        for key, btn in self._device_buttons.items():
            btn.setProperty("selected", key == self._selected_key)
            btn.style().unpolish(btn); btn.style().polish(btn)

    def _select_device(self, key: str):
        if not key:
            return
        try:
            self.api.select(key)
            self._selected_key = key
            self._apply_dev_styles()
            self.lblPedalStatus.setText(f"Вибраний пристрій: {self._device_buttons[key].text().splitlines()[0]}")
        except Exception as e:
            self.lblPedalStatus.setText(f"Не вдалося вибрати пристрій: {e}")

    #def on_stop(self):
    #    try:
    #        data = self.api.ext_stop()
    #        self.render(data)
    #    except Exception as e:
    #        self.lblPedalStatus.setText(f"Stop error: {e}")

    def render(self, st: dict):
        running = bool(st.get("external_running"))

        # 1) раз в 2 секунды подтягиваем конфиг
        now = time.monotonic()
        if (now - self._cfg_refresh_ts) > 2.0 or not self._devices:
            try:
                cfg = self.api.config()  # {"devices":[...], "selected": "key"|None}
                self._cfg_refresh_ts = now
                devices = cfg.get("devices") or []
                selected = cfg.get("selected") or None

                if devices != self._devices:
                    self._devices = devices
                    self._rebuild_devices(devices)

                # синхронизируем выбранное
                if selected != self._selected_key:
                    self._selected_key = selected
                    self._apply_dev_styles()

            except Exception as e:
                self.lblPedalStatus.setText(f"Помилка отримання конфігурації: {e}")

        # 2) доступность действий
        has_device = bool(self._selected_key)
        self.btnStart.setEnabled((not running) and has_device)
        #self.btnStop.setEnabled(running)

        # кнопки устройств блокируем во время работы
        for btn in self._device_buttons.values():
            btn.setEnabled(not running)

        self._refresh_events_log()

        # 3) статус
        if running:
            self.lblPedalStatus.setText('Програма працює. Перейдіть на вкладку РОБОТА та натисніть «ПУСК».')
        else:
            if has_device:
                name = ""
                if self._selected_key in self._device_buttons:
                    name = self._device_buttons[self._selected_key].text().splitlines()[0]
                self.lblPedalStatus.setText(f"Готово. Вибраний пристрій: {name}. Натисніть 'ПОЧАТИ РОБОТУ' для запуску програми.")
            else:
                self.lblPedalStatus.setText("Оберіть девайс, та натисніть 'ПОЧАТИ РОБОТУ'.")

    
    def on_device_changed(self, idx: int):
        key = self.cmbDevices.currentData()
        # игнорируем выбор placeholder
        if key in (None, ""):
            self._selected_key = None
            # серверу ничего не шлём
            return
        try:
            self.api.select(key)
            self._selected_key = key
            self.lblPedalStatus.setText(f"Вибраний пристрій: {self.cmbDevices.currentText()}")
        except Exception as e:
            self.lblPedalStatus.setText(f"Не вдалося вибрати пристрій: {e}")



# ================== Main Window ==================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AJAX TouchDesk (PyQt5)")
        self.setObjectName("root")
        self.api = ApiClient()
        self._was_running = False  # для отслеживания смены состояния external
        # Центральный контейнер
        self.frame = QFrame(); self.frame.setObjectName("rootFrame")
        self.frame.setProperty("state", "idle")
        self.setCentralWidget(self.frame)

        # Основной лэйаут (без логотипа!)
        root = QVBoxLayout(self.frame)
        #root.setSpacing(10)
        root.setContentsMargins(BORDER_W, BORDER_W, BORDER_W, BORDER_W)

        # Вкладки
        tabs = QTabWidget(); tabs.setObjectName("tabs")
        root.addWidget(tabs)

        self.tabWork    = WorkTab(self.api)
        self.tabStart   = StartTab(self.api)
        self.tabService = ServiceTab(self.api)

        tabs.addTab(self.tabWork,   "РОБОТА")     # idx 0
        tabs.addTab(self.tabStart,  "ЗАПУСК")    # idx 1
        tabs.addTab(self.tabService,"СЕРВІС")  # idx 2

        
        self.tabs = tabs
        # --- автопереход на START по специфическим ошибкам ---
        self._autoBackTimer = QTimer(self)
        self._autoBackTimer.setSingleShot(True)
        self._autoBackTimer.timeout.connect(self._switch_to_start_tab)

        # --- состояние показа сервис-вкладки ---
        self.service_visible = True   # временно True, чтобы убрать корректно
        self.service_unlocked = False
        self.ped_hold_start = None    # время начала удержания педали (float|None)

        # спрячем сервис сразу при старте
        self.hide_service_tab()

        try:
            self.tabs.setCurrentWidget(self.tabStart)
        except Exception:
            pass

        self.tabs.currentChanged.connect(self.on_tab_changed)
        self.on_tab_changed(self.tabs.currentIndex())
        self._was_running = False

         #мгновенный переход на Work после успешного старта в StartTab
        self.tabStart.on_started = lambda: self.tabs.setCurrentIndex(0)

        # ---------- ЛОГОТИП-ОВЕРЛЕЙ (абсолютно, не в layout) ----------
        self.logo = QLabel(self.frame)
        logo_path = os.path.join(os.path.dirname(__file__), "logo.png")
        pix = QPixmap(logo_path).scaledToHeight(60, Qt.SmoothTransformation)
        self.logo.setPixmap(pix)
        self.logo.setStyleSheet("background: transparent;")
        self.logo.adjustSize()
        self.logo.raise_()  # поверх всего

        # Отступы логотипа
        self._logo_margin_top = 65
        self._logo_margin_right = 50
        self._position_logo()  # первичное позиционирование


        # Таймер опроса API
        self.timer = QTimer(self); self.timer.setInterval(POLL_MS)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

        # Полноэкранный режим под тач
        self.showFullScreen()
        screen = QApplication.primaryScreen()
        if screen:
            size = screen.size()            # физическое разрешение фреймбуфера
            # фиксируем размер, чтобы layout не «догонял» что-то во время таб-свитча
            self.setFixedSize(size)
    
    def show_service_tab(self):
        """Показать вкладку SERVICE (если скрыта)."""
        if self.service_visible:
            return
        # вставим в конец (после WORK и START)
        idx = self.tabs.addTab(self.tabService, "СЕРВІС")
        self.tabs.setTabToolTip(idx, "Сервіс та діагностика")
        self.service_visible = True

    def _switch_to_start_tab(self):
        """Переключает на вкладку START (если есть)."""
        try:
            idx = self.tabs.indexOf(self.tabStart)
            if idx != -1 and self.tabs.currentIndex() != idx:
                self.tabs.blockSignals(True)
                self.tabs.setCurrentIndex(idx)
                self.tabs.blockSignals(False)
        except Exception:
            pass


    def hide_service_tab(self):
        """Скрыть вкладку SERVICE (если показана)."""
        idx = self.tabs.indexOf(self.tabService)
        if idx != -1:
            # если сейчас открыта — переключим на Work
            if self.tabs.currentIndex() == idx:
                self.tabs.setCurrentIndex(0)
            self.tabs.removeTab(idx)
        self.service_visible = False

    # Позиционирование рамки/бордера
    def set_border(self, state: str):
        self.frame.setProperty("state", state)
        self.frame.style().unpolish(self.frame); self.frame.style().polish(self.frame)

    # Перерисовка/логика статуса
    def refresh(self):
        # 1) Получаем статус
        try:
            st = self.api.status()
        except Exception:
            self.set_border("alarm")
            return     

        # 2) Базовые флаги
        running = bool(st.get("external_running"))
        sensors = st.get("sensors", {}) or {}
        # ВНИМАНИЕ: имя датчика педали должно совпадать с тем, что отдаёт /api/status
        # Если у тебя другое имя (например, "PEDAL"), поменяй ключ ниже:
        pedal_pressed = bool(sensors.get("PED_START"))

            # === ЧИТАЕМ ui_status.json, чтобы отреагировать на фазу error_no_air ===
        phase = ""
        try:
            if UI_STATUS_PATH.exists():
                ui = json.loads(UI_STATUS_PATH.read_text())
                phase = (ui.get("phase") or "").lower()
        except Exception:
            phase = ""

        # если цикл сообщил «нет воздуха» — запланировать автопрыжок на START через 5 сек
        if phase == "error_no_air":
            if not self._autoBackTimer.isActive():
                self._autoBackTimer.start(5000)   # 5 секунд
        else:
            # любая другая фаза — отменяем запланированный автопереход, если был
            if self._autoBackTimer.isActive():
                self._autoBackTimer.stop()


        # 3) ЛОГИКА ВИДИМОСТИ SERVICE
        if running:
            # Цикл запущен → прячем SERVICE до следующей разблокировки
            if self.service_visible:
                self.hide_service_tab()
            self.service_unlocked = False
            self.ped_hold_start = None
        else:
            # Цикл не запущен → разрешаем «секретное» открытие по удержанию педали 3 сек
            if not self.service_unlocked:
                if pedal_pressed:
                    # старт удержания
                    if self.ped_hold_start is None:
                        self.ped_hold_start = time.monotonic()
                    # удержание достигло 3.0 сек — разблокируем и показываем SERVICE
                    elif (time.monotonic() - self.ped_hold_start) >= 3.0:
                        self.service_unlocked = True
                        self.show_service_tab()
                else:
                    # педаль отпущена — сброс
                    self.ped_hold_start = None
            else:
                # уже разблокировано — гарантируем, что SERVICE видна
                if not self.service_visible:
                    self.show_service_tab()

        # 4) РЕНДЕР ВКЛАДОК (ровно один раз на тик)
        for tab in (self.tabWork, self.tabStart, self.tabService):
            try:
                tab.render(st)
            except Exception as e:
                # мягко логируем в консоль, чтобы не падать
                print(f"[UI] render error in {tab.__class__.__name__}: {e}")

        # 5) ВИЗУАЛЬНОЕ СОСТОЯНИЕ РАМКИ
        any_alarm = any(
            re.search(r"(alarm|emerg|fault|error|e_stop)", k, re.I) and bool(v)
            for k, v in sensors.items()
        )
        cycle_active = bool(
            st.get("cycle_busy")
            or st.get("cycle_active")
            or any(re.search(r"(pedal|emul|start_btn|cycle_running)", k, re.I) and bool(v)
                for k, v in sensors.items())
        )
        if any_alarm:
            self.set_border("alarm")
        elif cycle_active and running:
            self.set_border("ok")
        else:
            self.set_border("idle")

        # 6) БЛОКИРОВКА ТАБОВ ВО ВРЕМЯ РАБОТЫ
        # START таб включён только когда цикл не идёт
        self.tabs.setTabEnabled(1, not running)
        # SERVICE таб может быть скрыт — найдём индекс динамически
        svc_idx = self.tabs.indexOf(self.tabService)
        if svc_idx != -1:
            self.tabs.setTabEnabled(svc_idx, not running)

        # 7) Автопрыжок на WORK при переходе в RUNNING
        if running and not self._was_running and self.tabs.currentIndex() != 0:
            self.tabs.blockSignals(True)
            self.tabs.setCurrentIndex(0)
            self.tabs.blockSignals(False)

        # если цикл только что остановился — вернуться на START
        if (not running) and self._was_running:
            start_idx = self.tabs.indexOf(self.tabStart)
            if start_idx != -1 and self.tabs.currentIndex() != start_idx:
                self.tabs.blockSignals(True)
                self.tabs.setCurrentIndex(start_idx)
                self.tabs.blockSignals(False)

        # START всегда разрешена, когда не запущено
        self.tabs.setTabEnabled(1, not running)

        # SERVICE — по индексу, если присутствует
        svc_idx = self.tabs.indexOf(self.tabService)
        if svc_idx != -1:
            self.tabs.setTabEnabled(svc_idx, not running)


        # 8) Запомнить состояние RUNNING
        self._was_running = running


    # Абсолютное позиционирование логотипа
    def _position_logo(self):
        if not hasattr(self, "logo") or self.logo.pixmap() is None:
            return
        r = self.frame.rect()
        x = r.right() - self.logo.width() - self._logo_margin_right
        y = r.top() + self._logo_margin_top
        self.logo.move(x, y)

    # Держим логотип в углу при ресайзе
    def resizeEvent(self, event):
        self._position_logo()
        return super().resizeEvent(event)

    def on_tab_changed(self, idx: int):
        active = "work" if idx == 0 else ("start" if idx == 1 else "service")
        self.tabs.setProperty("active", active)
        self.tabs.style().unpolish(self.tabs)
        self.tabs.style().polish(self.tabs)



# ================== QSS ==================
APP_QSS = f"""
#root {{ background-color: #0f1115; }}
#rootFrame[state="ok"]    {{ border: {BORDER_W}px solid #1ac06b; }}
#rootFrame[state="idle"]  {{ border: {BORDER_W}px solid #f0b400; }}
#rootFrame[state="alarm"] {{ border: {BORDER_W}px solid #e5484d; }}

#devButton {{
    font-size: 20px; 
    font-weight: 700;
    text-align: left;
    padding: 14px 16px;
    border: 2px solid #3a4356; 
    border-radius: 14px;
    background: #2b3342; 
    color: #e8edf8;
}}
#devButton:hover {{ background: #354159; }}
#devButton[selected="true"] {{
    border-color: #1ac06b; 
    background: #153f2c;
    color: #e9ffee;
}}


#tabs::pane {{ border: none; }}
QTabBar::tab {{
    color: #cfd5e1;
    background: #1a1f29;
    padding: 18px 32px;   
    margin-right: 6px;
    border-top-left-radius: 10px;
    border-top-right-radius: 10px;
    font-size: 28px;
    font-weight: 700;
    min-height: 80px;
    min-width: 220px;
    border: 1px solid #2a3140;
}}
QTabBar::tab:selected {{ background: #242a36; color: white; }}

#card {{
    background: #1a1f29;
    border: 1px solid #2a3140;
    border-radius: 16px;
    color: #d8deea;
}}
#cardTitle {{ font-size: 18px; font-weight: 600; color: #eef3ff; }}

#bigButton {{
    font-size: 32px; font-weight: 700;
    background: #2b3342; color: #e8edf8; border: 2px solid #3a4356; border-radius: 18px;
}}
#bigButton[ok="true"]  {{ background: #153f2c; border-color: #1ac06b; color: #e9ffee; }}
#stopButton[ok="true"] {{ font-size: 32px; font-weight: 700; background: #3a1c1c; border-color: #e5484d; color: #ffe9e9; }}
#stopButton[ok="false"] {{ font-size: 32px; font-weight: 700; }}
#bigButton:pressed     {{ background: #354159; }}

#badge {{
    background: #2a3140; color: #dbe3f5; padding: 4px 10px; border-radius: 999px;
    font-weight: 600;
}}
#state {{ color: #cbd5e1; font-size: 16px; }}
QLabel#ok  {{ color: #1ac06b; font-weight: 700; }}
QLabel#off {{ color: #e5484d; font-weight: 700; }}

QPushButton {{
    background: #2b3342; color: #e8edf8;
    border: 1px solid #3a4356; border-radius: 10px; padding: 8px 14px;
}}

QTabBar::tab:!selected {{
    background: #1a1f29;
    color: #9aa7be;
    border-bottom: 4px solid transparent;
}}

QTabBar::tab:selected {{
    background: #242a36;
    color: #ffffff;
}}

QTabBar::tab:selected {{
    border-bottom: 6px solid #1ac06b;
}}
#tabs[active="work"] QTabBar::tab:selected {{
    border-bottom: 6px solid #1ac06b;}}

#tabs[active="service"] QTabBar::tab:selected {{
    border-bottom: 6px solid #3aa0ff;
}}
#tabs::pane {{ border: none; }}
QPushButton:disabled {{ opacity: .5; }}
QSpinBox, QLineEdit, QComboBox {{
    background: #1f2531; color: #dbe3f5; border: 1px solid #334157; border-radius: 8px; padding: 6px 8px;
}}
QTextEdit {{ background: #0f141c; color: #d3ddf0; border: 1px solid #334157; border-radius: 10px; }}
"""

def main():
    app = QApplication(sys.argv)
    from PyQt5.QtGui import QCursor # type: ignore
    # Инициализируем GPIO педали (если библиотека доступна) 
    try:
        gpio_pedal_init()
    except Exception as _e:
    # не падаем, просто сообщим в консоль
        print(f"[WARN] GPIO init failed: {_e}")
    app.setOverrideCursor(QCursor(Qt.BlankCursor))  # спрятать курсор
    app.setStyleSheet(APP_QSS)
    f = QFont(); f.setPixelSize(16); app.setFont(f)
    w = MainWindow(); w.show()
    # Корректная очистка GPIO при выходе
    app.aboutToQuit.connect(gpio_cleanup)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()