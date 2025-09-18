#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
# если нет переменных DISPLAY/WAYLAND_DISPLAY — поднимем eglfs
if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    os.environ.setdefault("QT_QPA_PLATFORM", "eglfs")
import socket
import os, sys, socket, re, time
import requests

from functools import partial

from PyQt5.QtCore import Qt, QTimer, QThread, QEvent, pyqtSignal as Signal
from PyQt5.QtGui import QFont # type: ignore
from PyQt5.QtGui import QPixmap # type: ignore
from PyQt5.QtWidgets import ( # type: ignore
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QTabWidget, QLabel, QPushButton, QFrame, QComboBox, QLineEdit,
    QTextEdit, QSpinBox, QSizePolicy, QInputDialog
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

# ================== Конфиг ==================
API_BASE = os.getenv("API_BASE", "http://127.0.0.1:8000/api")
POLL_MS   = 1000
BORDER_W  = 10

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
        self.btnPedal = big_button("Tighten the screws (pedal emulation)")
        self.btnKill  = big_button("STOP script")
        self.btnKill.setObjectName("stopButton")
        row.addWidget(self.btnPedal); row.addWidget(self.btnKill)
        root.addLayout(row, 1)

        self.stateLabel = QLabel("Status: unknown"); self.stateLabel.setObjectName("state")
        root.addWidget(self.stateLabel, 0, Qt.AlignLeft)

        self.btnPedal.clicked.connect(self.on_pedal)
        self.btnKill.clicked.connect(self.on_kill)

    def on_pedal(self):
        try:
            # Проверим, запущен ли внешний процесс — чтобы не жать в пустоту
            running = False
            try:
                st0 = self.api.status()
                running = bool(st0.get("external_running"))
            except Exception:
                pass

            if not running:
                self.stateLabel.setText(
                'The script is not running. First, click "Start program" on the "Start" tab.'
                )
                return

            # Убедимся, что listener поднят (порт 8765). Если нет — пару секунд подождём.
            t0 = time.time()
            while not is_port_open() and time.time() - t0 < 3.0:
                QApplication.processEvents()
                time.sleep(0.1)

            ok = send_start_trigger_with_retry()
            if ok:
                self.stateLabel.setText("START command sent (pedal emulation).")
            else:
                self.stateLabel.setText("Failed to send START (no response from loop).")

            # Обновим статус для подсветки кнопок
            try:
                st = self.api.status()
                self.render(st)
            except Exception:
                pass

        except Exception as e:
            self.stateLabel.setText(f"Error sending START: {e}")




    def on_kill(self):
        try:
            # если есть api/script/stop — добавь в ApiClient; здесь на всякий случай остановим external
            st = self.api.ext_stop()
            self.render(st)
        except Exception as e:
            self.stateLabel.setText(f"Stop error: {e}")

    def render(self, st: dict):
        running = bool(st.get("external_running"))
        self.stateLabel.setText("Status: " + ("PROGRAM RUNNING" if running else "PROGRAM STOPPED"))

        # === НОВОЕ: подсветка «Эмуляции педали», пока цикл ЗАНЯТ между нажатиями ===
        busy = bool(st.get("cycle_busy"))
        # когда busy=True — делаем кнопку зелёной
        self.btnPedal.setProperty("ok", busy)

        # актуальность «Стоп скрипта» как раньше
        self.btnKill.setProperty("ok", running)

        for w in (self.btnPedal, self.btnKill):
            w.style().unpolish(w); w.style().polish(w)



class VirtualKeyboard(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("vkRoot")
        self.setWindowFlags(Qt.Tool | Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.setParent(parent, Qt.Window)  # чтобы всегда быть поверх родительского окна
        self.setFrameShape(QFrame.NoFrame)

        self.target: QLineEdit | None = None
        lay = QVBoxLayout(self); lay.setContentsMargins(10,10,10,10); lay.setSpacing(8)

        # Минимальная раскладка
        rows = [
            list("1234567890"),
            list("QWERTYUIOP"),
            list("ASDFGHJKL"),
            list("ZXCVBNM"),
        ]
        grid = QGridLayout(); grid.setHorizontalSpacing(6); grid.setVerticalSpacing(6)
        r = 0
        for row in rows:
            c = 0
            for ch in row:
                b = QPushButton(ch)
                b.setFixedHeight(44)
                b.clicked.connect(lambda _, x=ch: self._insert(x))
                grid.addWidget(b, r, c); c += 1
            r += 1
        lay.addLayout(grid)

        # нижний ряд
        row = QHBoxLayout()
        self.btnSpace = QPushButton("Space")
        self.btnBack  = QPushButton("Backspace")
        self.btnClear = QPushButton("Clear")
        self.btnEnter = QPushButton("Enter")
        for b in (self.btnSpace, self.btnBack, self.btnClear, self.btnEnter):
            b.setFixedHeight(44); row.addWidget(b)
        lay.addLayout(row)

        self.btnSpace.clicked.connect(lambda: self._insert(" "))
        self.btnBack.clicked.connect(self._backspace)
        self.btnClear.clicked.connect(self._clear)
        self.btnEnter.clicked.connect(self._enter)

        self.setStyleSheet("""
        #vkRoot { background:#141923; border:2px solid #2a3140; border-radius:12px; }
        QPushButton { background:#2b3342; color:#e8edf8; border:1px solid #3a4356;
                      border-radius:8px; padding:6px 10px; font-size:16px; }
        QPushButton:pressed { background:#354159; }
        """)

        self.on_enter = None

    def _insert(self, s): 
        if self.target: self.target.insert(s)
    def _backspace(self):
        if self.target:
            t = self.target.text()
            if t: self.target.setText(t[:-1])
    def _clear(self):
        if self.target: self.target.clear()
    def _enter(self):
        self.hide()
        if callable(self.on_enter): self.on_enter()

    def show_for(self, line_edit: QLineEdit, parent_widget: QWidget):
        self.target = line_edit
        self.adjustSize()

        g = parent_widget.frameGeometry()
        kb_w = g.width() // 2          # половина ширины
        kb_h = g.height() // 3         # высота, например, 1/3 экрана
        x = g.x()                      # левый край
        y = g.y() + g.height() - kb_h  # прижать к низу
        self.setGeometry(x, y, kb_w, kb_h)
        self.show()
        self.raise_()                  # на самый верх


from PyQt5.QtWidgets import QDialog, QDialogButtonBox, QVBoxLayout, QLabel # type: ignore

class PasswordDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Password access")
        self.setObjectName("pwDialog")

        # Заголовок
        lbl = QLabel("Enter the password to access the Service")
        lbl.setObjectName("pwLabel")

        # Поле ввода
        self.edit = QLineEdit()
        self.edit.setEchoMode(QLineEdit.Password)
        self.edit.setMinimumHeight(80)    # высота ×4
        self.edit.setMinimumWidth(400)    # ширина побольше
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setObjectName("pwEdit")
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setModal(True)
        self.setAttribute(Qt.WA_TranslucentBackground)

        # Экранная клавиатура (наша)
        self.vkbd = VirtualKeyboard(self)
        self.vkbd.on_enter = self.accept
        self.edit.installEventFilter(self)

        # Кнопки
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        for b in buttons.buttons():
            b.setMinimumHeight(80)        # кнопки тоже крупные
            b.setMinimumWidth(200)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        layout.addWidget(lbl)
        layout.addWidget(self.edit)
        layout.addWidget(self.vkbd)   # клавиатура сразу под полем
        layout.addWidget(buttons)

        # применяем стили
        self.setStyleSheet("""
        #pwDialog {
            background-color: rgba(26, 31, 41, 220);
            border: 2px solid #2a3140;
                border-radius: 20px;
        }   
        #pwLabel {
            font-size: 28px;
            font-weight: bold;
            color: #eef3ff;
        }
        #pwEdit {
            font-size: 32px;
            font-weight: bold;
            color: #ffffff;
            background: #0f141c;
            border: 2px solid #3a4356;
            border-radius: 12px;
            padding: 10px;
        }
        QPushButton {
            font-size: 24px;
            font-weight: bold;
            background: #2b3342;
            color: #e8edf8;
            border: 2px solid #3a4356;
            border-radius: 12px;
            padding: 12px 20px;
        }
        QPushButton:pressed {
            background: #354159;
        }
        """)

    def eventFilter(self, obj, event):
        if obj is self.edit:
            if event.type() == QEvent.FocusIn:
                self.vkbd.show_for(self.edit, self)
            elif event.type() == QEvent.FocusOut:
                self.vkbd.hide()
        return super().eventFilter(obj, event)

    def get_password(self):
        if self.exec_() == QDialog.Accepted:
            return self.edit.text()
        return None


class ServiceTab(QWidget):
    def __init__(self, api: ApiClient, parent=None):
        super().__init__(parent)
        self.api = api
        self._relay_widgets = {}  # name -> (lblState, spin, btnOn, btnOff, btnPulse)

        root = QHBoxLayout(self); root.setContentsMargins(24,24,24,24); root.setSpacing(18)

        # Левая колонка
        left = QVBoxLayout(); left.setSpacing(18)

        self.sensorsCard = make_card("Сенсоры / концевики")
        self.sensorsGrid = QGridLayout(); self.sensorsGrid.setHorizontalSpacing(14); self.sensorsGrid.setVerticalSpacing(10)
        self.sensorsCard.layout().addLayout(self.sensorsGrid)
        left.addWidget(self.sensorsCard)

        self.relaysCard = make_card("Реле (ON/OFF/PULSE)")
        self.relaysGrid = QGridLayout(); self.relaysGrid.setHorizontalSpacing(8); self.relaysGrid.setVerticalSpacing(8)
        self.relaysCard.layout().addLayout(self.relaysGrid)
        left.addWidget(self.relaysCard, 1)

        # Правая колонка — Serial
        right = QVBoxLayout(); right.setSpacing(18)
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

        self.vkeyboard = VirtualKeyboard(self)
        self.vkeyboard.on_enter = self.send_serial
        self.edSend.installEventFilter(self)
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
            self.log_line(f"[HTTP {e.response.status_code}] manual control is not available WHEN THE PROGRAM IS RUNNING")
        except Exception as e:
            self.log_line(f"[ERROR] relay: {e}")

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
            self.vkeyboard.hide()
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
    
    def eventFilter(self, obj, event):
        if obj is self.edSend:
            if event.type() == QEvent.FocusIn:
                self.vkbd.show_for(self.edit, self)
            elif event.type() == QEvent.FocusOut:
                self.vkbd.hide()
        return super().eventFilter(obj, event)


class StartTab(QWidget):
    """Третья вкладка: большие кнопки Start/Stop external."""
    def __init__(self, api: ApiClient, parent=None):
        super().__init__(parent)
        self.api = api
        self.on_started = None  # коллбек задаст MainWindow

        root = QVBoxLayout(self); root.setContentsMargins(24,24,24,24); root.setSpacing(18)
        row = QHBoxLayout(); row.setSpacing(18)
        self.btnStart = big_button("START program")
        self.btnStop  = big_button("STOP program")
        self.btnStop.setObjectName("stopButton")
        row.addWidget(self.btnStart); row.addWidget(self.btnStop)
        root.addLayout(row, 1)

        self.stateLabel = QLabel("Status: unknown"); self.stateLabel.setObjectName("state")
        root.addWidget(self.stateLabel, 0, Qt.AlignLeft)

        self.btnStart.clicked.connect(self.on_start)
        self.btnStop.clicked.connect(self.on_stop)

    def on_start(self):
        try:
            self.stateLabel.setText("Launching the program…")
            self.api.ext_start()  # только стартуем процесс (как в веб UI /api/ext/start)
            # покажем подсказку оператору
            self.stateLabel.setText(
                'The program is running. To begin working, click "START" on the Work tab.'
            )
            # дальше переключать вкладку НЕ будем — авто-переброс на Work произойдёт,
            # когда external_running станет True (логика уже есть в refresh()).
        except Exception as e:
            self.stateLabel.setText(f"Failed to start the program: {e}")



    def on_stop(self):
        try:
            data = self.api.ext_stop()
            self.render(data)
        except Exception as e:
            self.stateLabel.setText(f"Stop error: {e}")

    def render(self, st: dict):
        running = bool(st.get("external_running"))
        self.stateLabel.setText("Статус: " + ("PROGRAM RUNNING" if running else "PROGRAM STOPPED"))
        self.btnStart.setProperty("ok", running)
        self.btnStop.setProperty("ok", not running)
        for w in (self.btnStart, self.btnStop):
            w.style().unpolish(w); w.style().polish(w)



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
        root.setContentsMargins(BORDER_W, BORDER_W, BORDER_W, BORDER_W)

        # Вкладки
        tabs = QTabWidget(); tabs.setObjectName("tabs")
        root.addWidget(tabs)

        self.tabWork    = WorkTab(self.api)
        self.tabStart   = StartTab(self.api)
        self.tabService = ServiceTab(self.api)

        tabs.addTab(self.tabWork,   "WORK")     # idx 0
        tabs.addTab(self.tabStart,  "START")    # idx 1
        tabs.addTab(self.tabService,"SERVICE")  # idx 2

        self.tabs = tabs
        self.tabs.currentChanged.connect(self.check_service_tab)
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

    # Позиционирование рамки/бордера
    def set_border(self, state: str):
        self.frame.setProperty("state", state)
        self.frame.style().unpolish(self.frame); self.frame.style().polish(self.frame)

    # Перерисовка/логика статуса
    def refresh(self):
        try:
            st = self.api.status()
        except Exception:
            self.set_border("alarm")
            return

        # обновление вкладок
        self.tabWork.render(st)
        self.tabStart.render(st)
        self.tabService.render(st)

        running = bool(st.get("external_running"))
        sensors = st.get("sensors", {})
        any_alarm = any(re.search(r"(alarm|emerg|fault|error|e_stop)", k, re.I) and v for k, v in sensors.items())

        if running:
            try:
                st = self.api.status()
            except Exception:
                self.set_border("alarm")
                return

            # обновление вкладок
            self.tabWork.render(st)
            self.tabStart.render(st)
            self.tabService.render(st)

            # ----------------------------
            # логика рамки
            # ----------------------------
            sensors = st.get("sensors", {}) or {}
            any_alarm = any(
                re.search(r"(alarm|emerg|fault|error|e_stop)", k, re.I) and bool(v)
                for k, v in sensors.items()
            )

            cycle_active = bool(
                st.get("cycle_busy") or
                st.get("cycle_active") or
                any(
                    re.search(r"(pedal|emul|start_btn|cycle_running)", k, re.I) and bool(v)
                    for k, v in sensors.items()
                )
            )

            if any_alarm:
                self.set_border("alarm")   # красная
            elif cycle_active:
                self.set_border("ok")      # зелёная
            else:
                self.set_border("idle")    # жёлтая


            # если только что перешли в RUNNING — СНАЧАЛА переключимся на Work,
            # а уже потом отключим вкладки, чтобы Qt не дёргал текущий индекс
            running = bool(st.get("external_running"))
            if not self._was_running and self.tabs.currentIndex() != 0:
                self.tabs.blockSignals(True)              # чтобы не срабатывал check_service_tab
                self.tabs.setCurrentIndex(0)
                self.tabs.blockSignals(False)

            # теперь блокируем Старт и Service
            self.tabs.setTabEnabled(1, False)
            self.tabs.setTabEnabled(2, False)

        else:
            self.tabs.setTabEnabled(1, True)
            self.tabs.setTabEnabled(2, True)

        self._was_running = running

    # Пароль на вкладку Service
    def check_service_tab(self, idx: int):
        if idx == 2:
            # если вкладка Service отключена (при external_running=True) — тихо откатиться на Work
            if not self.tabs.isTabEnabled(2):
                self.tabs.blockSignals(True)
                self.tabs.setCurrentIndex(0)
                self.tabs.blockSignals(False)
                return

            dlg = PasswordDialog(self)
            pw = dlg.get_password()
            if pw != "1234":
                self.tabs.blockSignals(True)
                self.tabs.setCurrentIndex(0)
                self.tabs.blockSignals(False)



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
    f = QFont(); f.setPointSize(12); app.setFont(f)
    w = MainWindow(); w.show()
    # Корректная очистка GPIO при выходе
    app.aboutToQuit.connect(gpio_cleanup)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
