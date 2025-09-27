#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
import subprocess
import sys
import os
import signal
import socket
from pathlib import Path
import json
import json, yaml
from pathlib import Path
try:
    import socket
except Exception:
    socket = None
from functools import wraps
from flask import Flask, request, jsonify, Response

from cycle_onefile import IOController, RELAY_PINS, SENSOR_PINS

BUSY_FLAG = "/tmp/screw_cycle_busy"
CFG_PATH = Path(__file__).with_name("devices.yaml")
SELECTED_PATH = Path("/tmp/selected_device.json")
EVENT_LOG_PATH = Path("/tmp/screw_events.jsonl")
# ---------------------- Инициализация ----------------------
app = Flask(__name__)

io_lock = threading.Lock()
io: IOController | None = IOController()  # контроллер GPIO (можем временно освободить)

# Внутренний «цикл» веб-панели (если ты его использовал раньше) — оставим выключенным.
# Мы запускаем внешний скрипт как отдельный процесс.
cycle_thread = None   # не используется, оставлено для совместимости
cycle_running = False # не используется, оставлено для совместимости

# Внешний процесс (cycle_onefile.py)
ext_proc: subprocess.Popen | None = None

TIMEOUT_SEC = 5.0  # базовый таймаут для ожидания датчиков (если понадобится)

def load_devices_list():
    try:
        text = CFG_PATH.read_text(encoding="utf-8")
    except FileNotFoundError:
        return []
    data = yaml.safe_load(text) or {}
    devs = data.get("devices", []) if isinstance(data, dict) else []
    out = []
    for d in devs:
        try:
            out.append({"key": d["key"], "name": d["name"], "holes": d.get("holes")})
        except Exception:
            continue
    return out


def get_selected_key():
    try:
        return json.loads(SELECTED_PATH.read_text())["key"]
    except Exception:
        return None

def set_selected_key(key: str):
    SELECTED_PATH.write_text(json.dumps({"key": key, "ts": int(time.time())}))


def with_io_lock(fn):
    @wraps(fn)
    def wrapper(*args, **kwargs):
        with io_lock:
            return fn(*args, **kwargs)
    return wrapper

# ---------------------- GPIO helpers ----------------------
def _sensor_state(name: str) -> bool:
    if io is None:
        return False
    return io.sensor_state(name)

def _set_relay(name: str, on: bool):
    if io is None:
        raise RuntimeError("GPIO not available (external script running)")
    io.set_relay(name, on)

def _pulse(name: str, ms: int):
    if io is None:
        raise RuntimeError("GPIO not available (external script running)")
    io.pulse(name, ms=ms)

def send_start_trigger(host="127.0.0.1", port=8765, payload=b"START\n", timeout=0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.sendall(payload)
            s.settimeout(timeout)
            resp = s.recv(64)
            return b"OK" in resp.upper()
    except Exception:
        return False



# ---------------------- External script control ----------------------
def ext_is_running() -> bool:
    return ext_proc is not None and (ext_proc.poll() is None)

@with_io_lock
def ext_start() -> bool:
    global io, ext_proc
    if ext_is_running():
        return True

    # требуем выбранного устройства
    sel = get_selected_key()
    if not sel:
        return False  # UI поймает и покажет предупреждение

    if io is not None:
        try: io.cleanup()
        except Exception: pass
        io = None

    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cycle_onefile.py")
    ext_proc = subprocess.Popen(
        [sys.executable, script_path, "--device", sel],  # <-- здесь
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
    )
    return True

@with_io_lock
def ext_stop() -> bool:
    """Останавливаем внешний процесс и восстанавливаем GPIO в web_ui."""
    global io, ext_proc
    if not ext_is_running():
        # уже остановлен — просто убедиться, что GPIO восстановлены
        if io is None:
            io = IOController()
        return True

    try:
        # Послать SIGINT (как Ctrl+C), дать шанс корректно завершиться
        ext_proc.send_signal(signal.SIGINT)
        try:
            ext_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            # Жестко завершим
            ext_proc.kill()
            ext_proc.wait(timeout=2.0)
    except Exception:
        pass
    finally:
        ext_proc = None

    # Восстановить GPIO в веб-панели
    if io is None:
        io = IOController()
    return True

# ---------------------- Status builder ----------------------
def build_status():
    external = ext_is_running()
    if external or io is None:
        relays, sensors = {}, {}
    else:
        with io_lock:
            relays = dict(io.relays)
            sensors = {name: io.sensor_state(name) for name in SENSOR_PINS.keys()}

    return {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "relays": relays,
        "sensors": sensors,
        "relay_names": list(RELAY_PINS.keys()),
        "sensor_names": list(SENSOR_PINS.keys()),
        "external_running": external,
        "cycle_busy": os.path.exists(BUSY_FLAG),
        "devices": load_devices_list(),
        "selected_device": get_selected_key(),
    }


# ---------------------- API ----------------------
@app.route("/api/status", methods=["GET"])
def api_status():
    return jsonify(build_status())

@app.route("/api/relay", methods=["POST"])
def api_relay():
    # Блокируем ручное управление, если внешний скрипт запущен
    if ext_is_running() or io is None:
        return jsonify({"error": "external_running", "message": "Запущено зовнішній скрипт — ручне керування тимчасово недоступне."}), 409

    try:
        data = request.get_json(force=True)
    except Exception:
        return jsonify({"error": "invalid json"}), 400

    name = data.get("name")
    action = data.get("action")
    ms = int(data.get("ms", 150))

    if name not in RELAY_PINS:
        return jsonify({"error": f"unknown relay '{name}'"}), 400

    with io_lock:
        if action == "on":
            _set_relay(name, True)
        elif action == "off":
            _set_relay(name, False)
        elif action == "pulse":
            _pulse(name, ms=ms)
        else:
            return jsonify({"error": "action must be 'on' | 'off' | 'pulse'"}), 400

    return jsonify(build_status())

@app.route("/api/ext/start", methods=["POST"])
def api_ext_start():
    ok = ext_start()
    # даём процессу стартануть
    time.sleep(0.1)
    return jsonify(build_status())

@app.route("/api/ext/stop", methods=["POST"])
def api_ext_stop():
    ok = ext_stop()
    # дать GPIO переинициализироваться
    time.sleep(0.1)
    return jsonify(build_status())

@app.route("/api/last_exit")
def api_last_exit():
    p = Path("/tmp/last_exit.json")
    if p.exists():
        try:
            return jsonify({"ok": True, "data": json.loads(p.read_text(encoding="utf-8"))})
        except Exception:
            return jsonify({"ok": False, "err": "read_failed"})
    return jsonify({"ok": True, "data": None})


@app.route("/api/trigger/start", methods=["POST"])
def api_trigger_start():
    if not ext_is_running():
        return jsonify({"error":"not_running","message":"Зовнішній скрипт не запущено"}), 409
    ok = send_start_trigger()
    if not ok:
        return jsonify({"error":"connect","message":"Не вдалося відправити команду START у цикл"}), 502
    # вернём свежий статус
    time.sleep(0.1)
    return jsonify(build_status())

@app.route("/api/select", methods=["POST"])
def api_select():
    data = request.get_json(force=True)
    key = data.get("key")
    devs = load_devices_list()
    if not any(d["key"] == key for d in devs):
        return jsonify({"error": "unknown device"}), 400
    set_selected_key(key)
    return jsonify({"ok": True, "selected": key})

@app.route("/api/config", methods=["GET"])
def api_config():
    return jsonify({"devices": load_devices_list(), "selected": get_selected_key()})

@app.route("/api/events")
def api_events():
    limit = int(request.args.get("limit", "120"))
    events = []
    try:
        lines = EVENT_LOG_PATH.read_text(encoding="utf-8").splitlines()
        for ln in lines[-limit:]:
            try:
                events.append(json.loads(ln))
            except Exception:
                continue
    except FileNotFoundError:
        events = []
    except Exception:
        events = []
    return jsonify({"events": events})


# ---------------------- UI ----------------------
INDEX_HTML = """<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<title>RPi IO Панель</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,"Helvetica Neue",Arial,sans-serif;margin:20px;line-height:1.4}
  h1{margin:0 0 12px}
  .row{display:flex;gap:24px;flex-wrap:wrap}
  .card{border:1px solid #ddd;border-radius:12px;padding:16px;box-shadow:0 2px 6px rgba(0,0,0,.05);min-width:280px}
  table{width:100%;border-collapse:collapse}
  th,td{padding:8px;border-bottom:1px solid #eee;font-size:14px}
  .ok{color:#0a7a1f;font-weight:600}
  .off{color:#a00;font-weight:600}
  .btn{padding:6px 10px;border:1px solid #ccc;border-radius:8px;background:#fafafa;cursor:pointer}
  .btn:hover{background:#f0f0f0}
  .small{font-size:12px;color:#666}
  .controls{display:flex;gap:8px;flex-wrap:wrap}
  input[type=number]{width:80px;padding:6px;border:1px solid #ccc;border-radius:8px}
  .badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;background:#eee}
  .pill{display:inline-block;padding:2px 10px;border-radius:999px;font-size:12px}
  .pill.green{background:#d9f5df;color:#0a7a1f;border:1px solid #a7e5b2}
  .pill.gray{background:#eee;color:#333;border:1px solid #ddd}
  .pill.blue{background:#cfe7ff;color:#0c4a8a;border:1px solid #99c2ff}
  .muted{color:#777;font-size:12px}
  .disabled{opacity:.5;pointer-events:none}
</style>
</head>
<body>
  <h1>RPi IO Панель</h1>
  <div id="exitBanner" style="display:none;padding:8px;border-radius:8px;background:#ffd6d6;color:#7a0000;margin:8px 0;"></div>
  <div class="small" id="statusTime"></div>

  <div class="row">
    <div class="card" style="flex:1">
      <h3>Зовнішній скрипт: cycle_onefile.py</h3>
      <div id="extState" class="muted">Статус: невідомо</div>
      <div class="controls" style="margin-top:8px">
        <select id="deviceSelect"></select>
        <button id="btnExtStart" class="btn">Запустити</button>
        <button id="btnExtStop"  class="btn">Зупинити</button>
        <button id="btnCmdStart" class="btn">Надіслати START (команда)</button>
      </div>
<div class="muted">Перед запуском виберіть пристрій.</div>
      <div class="muted" style="margin-top:8px">Коли зовнішній скрипт запущено, веб-панель не чіпає GPIO і ручне керування недоступне.</div>
    </div>

    <div class="card" style="flex:1">
      <h3>Датчики</h3>
      <table id="sensorsTbl">
        <thead><tr><th>Назва</th><th>Стан</th></tr></thead>
        <tbody></tbody>
      </table>
    </div>

    <div class="card" style="flex:1">
      <h3>Реле</h3>
      <table id="relaysTbl">
        <thead><tr><th>Назва</th><th>Стан</th><th>Керування</th></tr></thead>
        <tbody></tbody>
      </table>
      <div class="muted">Якщо зовнішній скрипт запущено — кнопки будуть вимкнені.</div>
    </div>
  </div>
<h3>Події</h3>
<div id="events" class="events"></div>

<style>
  .events { max-height: 280px; overflow:auto; border:1px solid #ddd; border-radius:8px; padding:8px; font-family: ui-monospace,monospace; background:#fafafa;}
  .ev { display:flex; gap:8px; padding:4px 0; border-bottom:1px dashed #eee; }
  .ev .ts { color:#888; width:150px; flex:0 0 auto;}
  .ev .lvl { width:70px; text-align:center; border-radius:6px; padding:0 6px; flex:0 0 auto;}
  .ev .lvl.INFO { background:#eef; }
  .ev .lvl.WARN { background:#ffeec2; }
  .ev .lvl.ERROR { background:#ffd6d6; }
  .ev .lvl.ALARM { background:#ffc6c6; font-weight:700;}
  .ev .code { width:120px; color:#555; flex:0 0 auto;}
  .ev .msg { flex:1; }
</style>

  

<script>


async function getStatus(){
  const res = await fetch('/api/status');
  if(!res.ok) throw new Error('status HTTP '+res.status);
  return await res.json();
}

async function postRelay(name, action, ms){
  const payload = { name, action };
  if(action==='pulse' && ms) payload.ms = ms;
  const res = await fetch('/api/relay', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(payload)
  });
  if(res.status===409){
    const data = await res.json();
    alert(data.message || 'Зовнішній скрипт запущено. Ручне керування недоступне.');
    return null;
  }
  if(!res.ok) throw new Error('relay HTTP '+res.status);
  return await res.json();
}

async function postExt(action){
  const url = action==='start' ? '/api/ext/start' : '/api/ext/stop';
  const res = await fetch(url, {method:'POST'});
  if(!res.ok) throw new Error('ext HTTP '+res.status);
  return await res.json();
}

async function postTriggerStart(){
  const res = await fetch('/api/trigger/start', {method:'POST'});
  if(res.status===409){
    const data = await res.json();
    alert(data.message || 'Зовнішній скрипт не запущено');
    return null;
  }
  if(!res.ok) throw new Error('trigger HTTP '+res.status);
  return await res.json();
}

async function loadConfig(){
  const res = await fetch('/api/config');
  if(!res.ok) throw new Error('config HTTP '+res.status);
  const data = await res.json();
  const sel = document.getElementById('deviceSelect');
  sel.innerHTML = '';
  const ph = document.createElement('option');
  ph.value = ''; ph.textContent = '— виберіть пристрій —';
  sel.appendChild(ph);
  (data.devices || []).forEach(d=>{
    const o = document.createElement('option');
    o.value = d.key; o.textContent = d.name;
    sel.appendChild(o);
  });
  if(data.selected){
    sel.value = data.selected;
  }
  // обновим состояние кнопок
  renderExternal(false, !!data.selected);
}

async function postSelect(key){
  const res = await fetch('/api/select', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({key})
  });
  if(!res.ok){ alert('Не вдалося вибрати пристрій'); }
}

function renderExternal(isRunning, hasDevice){
  const state = document.getElementById('extState');
  state.innerHTML = 'Статус: ' + (isRunning ? '<span class="pill blue">EXTERNAL RUNNING</span>' : '<span class="pill gray">STOPPED</span>');
  document.getElementById('relaysTbl').classList.toggle('disabled', isRunning);
  document.getElementById('btnCmdStart').disabled = !isRunning;
  // запрещаем запуск без выбранного устройства
  document.getElementById('btnExtStart').disabled = !hasDevice || isRunning;
}

function render(data){
  document.getElementById('statusTime').textContent = 'Оновлено: ' + data.time;
  renderExternal(!!data.external_running, !!data.selected_device);

  // sensors
  const sbody = document.querySelector('#sensorsTbl tbody');
  sbody.innerHTML = '';
  for(const name of data.sensor_names){
    const val = data.sensors[name];
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><span class="badge">${name}</span></td>
      <td>${val ? '<span class="ok">CLOSE</span>' : '<span class="off">OPEN</span>'}</td>
    `;
    sbody.appendChild(tr);
  }

  // relays
  const rbody = document.querySelector('#relaysTbl tbody');
  rbody.innerHTML = '';
  for(const name of data.relay_names){
    const val = data.relays[name];
    const tr = document.createElement('tr');
    const pulseId = 'pulse_'+name;
    tr.innerHTML = `
      <td><span class="badge">${name}</span></td>
      <td>${val ? '<span class="ok">ON</span>' : '<span class="off">OFF</span>'}</td>
      <td>
        <div class="controls">
          <button class="btn" onclick="cmd('${name}','on')">ON</button>
          <button class="btn" onclick="cmd('${name}','off')">OFF</button>
          <input type="number" id="${pulseId}" min="20" value="150" title="Pulse, ms">
          <button class="btn" onclick="cmd('${name}','pulse', document.getElementById('${pulseId}').value)">PULSE</button>
        </div>
      </td>
    `;
    rbody.appendChild(tr);
  }

  // disable inputs if external running
  const disabled = !!data.external_running;
  document.querySelectorAll('#relaysTbl button, #relaysTbl input').forEach(el=>{ el.disabled = disabled; });
}

async function loadLastExit(){
  try{
    const r = await fetch('/api/last_exit');
    const j = await r.json();
    const b = document.getElementById('exitBanner');
    if (j.ok && j.data){
      b.style.display = 'block';
      const d = j.data;
      b.textContent = `[${d.ts}] ${d.msg}`;
    } else {
      b.style.display = 'none';
    }
  }catch(e){
    // молча
  }
}



async function refresh(){
  try{
    const data = await getStatus();
    render(data);
  }catch(e){
    console.error(e);
  }
}
async function cmd(name, action, ms){
  const data = await postRelay(name, action, ms?parseInt(ms,10):undefined);
  if(data) render(data);
}

let lastPopupTs = null;

document.getElementById('deviceSelect').addEventListener('change', async (e)=>{
  const key = e.target.value || null;
  if(key){ await postSelect(key); }
  // сразу перерисуем статус — чтобы активировать кнопку Start
  render(await getStatus());
});

document.getElementById('btnExtStart').addEventListener('click', async ()=>{
  const sel = document.getElementById('deviceSelect').value;
  if(!sel){ alert('Виберіть пристрій зі списку'); return; }
  try{ render(await postExt('start')); }catch(e){ alert('Помилка запуску: '+e.message); }
});
document.getElementById('btnExtStop').addEventListener('click', async ()=>{
  try{ render(await postExt('stop')); }catch(e){ alert('Помилка зупинки: '+e.message); }
});
document.getElementById('btnCmdStart').addEventListener('click', async ()=>{
  try{
    const data = await postTriggerStart();
    if(data) render(data);
  }catch(e){ alert('Помилка команди START: '+e.message); }
});

window.addEventListener('load', async ()=>{
  await loadConfig();          // ← заполняем выпадающий список
  render(await getStatus());   // ← рисуем UI
  await loadLastExit();       // ← рисуем баннер последнего выхода
  setInterval(refresh, 1000);
});

async function loadEvents(){
  try{
    const res = await fetch('/api/events?limit=120');
    if(!res.ok) return;
    const data = await res.json();
    const box = document.getElementById('events');
    box.innerHTML = '';
    (data.events || []).forEach(ev=>{
      const div = document.createElement('div');
      div.className = 'ev';
      div.innerHTML = `
        <div class="ts">${ev.ts || ''}</div>
        <div class="lvl ${ev.level || ''}">${ev.level || ''}</div>
        <div class="code">${ev.code || ''}</div>
        <div class="msg">${ev.msg || ''}</div>
      `;
      box.appendChild(div);
    });
    // Всплывашка при событиях с extra.popup === true (показаем один раз на ts)
    const popupEv = (data.events || []).reverse().find(ev => ev && ev.extra && ev.extra.popup);
    if (popupEv && popupEv.ts && popupEv.ts !== lastPopupTs) {
      alert(popupEv.msg || 'Увага!');
      lastPopupTs = popupEv.ts;
    }
    box.scrollTop = box.scrollHeight; // автоскролл вниз
  }catch(e){ console.error(e); }
}

// в onload после твоих loadConfig()/render()
window.addEventListener('load', async ()=>{
  // у тебя уже есть: await loadConfig(); render(await getStatus());
  await loadEvents();
  await loadLastExit();
  // периодические обновления:
  setInterval(loadEvents, 1500);
});
</script>

</body>
</html>
"""

@app.route("/", methods=["GET"])
def index():
    return Response(INDEX_HTML, mimetype="text/html")

# ---------------------- Запуск ----------------------
def main():
    # При желании — отключить болтливость dev-сервера:
    # import logging
    # logging.getLogger('werkzeug').disabled = True
    app.run(host="0.0.0.0", port=8000, debug=False, threaded=False, use_reloader=False)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        with io_lock:
            # если внешний процесс всё ещё жив — не трогаем GPIO
            if not ext_is_running() and io is not None:
                io.cleanup()
