#!/usr/bin/env python3
import argparse, sys, time, threading
from typing import Optional
import serial

DEFAULT_BAUD = 115200

class GLink:
    def __init__(self, port:str, baud:int=DEFAULT_BAUD, timeout:float=2.0, eol:str="\n"):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.eol = eol
        self.ser: Optional[serial.Serial] = None
        self._rx_thread = None
        self._rx_running = False
        self._rx_buffer = []
        self._rx_lock = threading.Lock()

    def open(self):
        # Отключаем автосброс по DTR/RTS и открываем порт без лишних пауз
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        # НЕ спим 2 секунды
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None

    def _rx_worker(self):
        assert self.ser
        while self._rx_running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                line = ""
            if line:
                with self._rx_lock:
                    self._rx_buffer.append(line)

    def start_reader(self):
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self._rx_thread.start()

    def stop_reader(self):
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None

    def send(self, cmd:str, wait_ok:bool=True, print_io:bool=True, timeout_s:float=5.0):
        """Отправить команду и подождать 'ok' (или 'err ...'). Возвращает список строк ответа."""
        if not self.ser: raise RuntimeError("Port is not open")
        full = cmd.strip() + self.eol
        if print_io:
            print(f">> {cmd}")
        self.ser.write(full.encode())
        out = []
        if not wait_ok:
            return out
        # ждём ok/err
        t0 = time.time()
        while True:
            # собираем всё, что уже пришло
            with self._rx_lock:
                while self._rx_buffer:
                    ln = self._rx_buffer.pop(0)
                    out.append(ln)
                    if print_io:
                        print(ln)
                    # наши окончания транзакции
                    if ln == "ok" or ln.startswith("ok "):
                        return out
                    if ln.startswith("err"):
                        # всё равно вернём, но бросим исключение
                        raise RuntimeError("\n".join(out))
            if time.time() - t0 > timeout_s:
                raise TimeoutError(f"Timeout waiting OK for '{cmd}'")
            time.sleep(0.01)

def build_parser():
    p = argparse.ArgumentParser(description="RPi ↔ Arduino (RAMPS) CLI for your XY table")
    p.add_argument("--port", "-p", default="/dev/ttyACM0", help="Serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help="Baudrate")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("repl", help="Interactive mode: keep port open and send lines")

    sub.add_parser("ping", help="PING → PONG")
    sub.add_parser("status", help="M114/M119")

    home = sub.add_parser("home", help="Home axes")
    home.add_argument("axes", nargs="*", choices=["X","Y","XY"], default=["XY"],
                      help="X, Y or XY (default XY)")

    move = sub.add_parser("move", help="Absolute move G X.. Y.. F.. (mm/min)")
    move.add_argument("--x", type=float, required=True)
    move.add_argument("--y", type=float, required=True)
    move.add_argument("--f", type=float, default=1200.0)

    dx = sub.add_parser("dx", help="Relative jog X by mm (DX)")
    dx.add_argument("mm", type=float)
    dx.add_argument("--f", type=float, default=600.0)

    dy = sub.add_parser("dy", help="Relative jog Y by mm (DY)")
    dy.add_argument("mm", type=float)
    dy.add_argument("--f", type=float, default=600.0)

    sub.add_parser("zero", help="Go to (0,0)")

    ssteps = sub.add_parser("set-steps", help="SET STEPS X.. Y..")
    ssteps.add_argument("--x", type=float, required=True)
    ssteps.add_argument("--y", type=float, required=True)

    slim = sub.add_parser("set-lim", help="SET LIM X.. Y..")
    slim.add_argument("--x", type=float, required=True)
    slim.add_argument("--y", type=float, required=True)

    raw = sub.add_parser("raw", help="Send raw line to firmware")
    raw.add_argument("line", nargs="+")
    return p

def main():
    args = build_parser().parse_args()
    gl = GLink(args.port, args.baud)
    try:
        gl.open()
        gl.start_reader()

        # небольшой "рукопожатие"
        try:
            gl.send("PING")
        except Exception:
            # не все версии печатают PONG — просто игнорируем
            pass

        if args.cmd == "ping":
            gl.send("PING")

        elif args.cmd == "status":
            gl.send("M114")
            gl.send("M119")

        elif args.cmd == "home":
            axes = args.axes
            if axes == ["XY"]: 
                gl.send("G28")
            elif axes == ["X"]:
                gl.send("G28 X")
            elif axes == ["Y"]:
                gl.send("G28 Y")
            else:
                # допустим 'X Y'
                if "X" in axes: gl.send("G28 X")
                if "Y" in axes: gl.send("G28 Y")

        elif args.cmd == "move":
            gl.send(f"G X{args.x} Y{args.y} F{args.f}")

        elif args.cmd == "dx":
            gl.send(f"DX {args.mm:+g} F{args.f}")

        elif args.cmd == "dy":
            gl.send(f"DY {args.mm:+g} F{args.f}")

        elif args.cmd == "zero":
            gl.send("ZERO")

        elif args.cmd == "set-steps":
            gl.send(f"SET STEPS X{args.x} Y{args.y}")

        elif args.cmd == "set-lim":
            gl.send(f"SET LIM X{args.x} Y{args.y}")

        elif args.cmd == "raw":
            line = " ".join(args.line)
            # для "G ..." отвечаем до ok; для "DX/DY" тоже
            gl.send(line)
        elif args.cmd == "repl":
            print("Connected. Type commands (e.g. G28, M119, G X10 Y5 F3000). Ctrl+C to exit.")
            while True:
                try:
                    line = input("> ").strip()
                    if not line: 
                        continue
                    gl.send(line, wait_ok=True, print_io=True, timeout_s=30.0)
                except (KeyboardInterrupt, EOFError):
                    print("\nBye")
                    break
        else:
            print("Unknown command", file=sys.stderr)
            sys.exit(2)

    except (RuntimeError, TimeoutError) as e:
        print(f"\nERROR: {e}\n", file=sys.stderr)
        sys.exit(1)
    finally:
        try:
            gl.stop_reader()
            gl.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
