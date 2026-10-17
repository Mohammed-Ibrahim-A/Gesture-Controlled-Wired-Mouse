# AeroClick receiver for Windows - USB serial version (aeroclick_receiver.py)
# 1) pip install pyserial          (one time)
# 2) Plug in the board, CLOSE the Arduino Serial Monitor
# 3) python aeroclick_receiver.py

import ctypes
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial is missing. Run:  pip install pyserial")

if sys.platform != "win32":
    raise SystemExit("This receiver only works on Windows.")

# Leave as None to auto-detect the board, or set e.g. "COM5"
PORT = None

# Extra speed control on top of the firmware's SENSITIVITY (1.0 = unchanged)
SPEED = 1.0

user32 = ctypes.windll.user32  # type: ignore[attr-defined]

MOVE = 0x0001
LEFTDOWN, LEFTUP = 0x0002, 0x0004
RIGHTDOWN, RIGHTUP = 0x0008, 0x0010
WHEEL = 0x0800

_rx = 0.0
_ry = 0.0


def mouse_move(dx, dy):
    global _rx, _ry
    _rx += dx * SPEED
    _ry += dy * SPEED
    ix, iy = int(_rx), int(_ry)
    _rx -= ix
    _ry -= iy
    if ix or iy:
        user32.mouse_event(MOVE, ix & 0xFFFFFFFF, iy & 0xFFFFFFFF, 0, 0)


def mouse_wheel(notches):
    user32.mouse_event(WHEEL, 0, 0, (notches * 120) & 0xFFFFFFFF, 0)


def click(down, up):
    user32.mouse_event(down, 0, 0, 0, 0)
    time.sleep(0.02)
    user32.mouse_event(up, 0, 0, 0, 0)


def find_port():
    if PORT:
        return PORT
    ports = list(list_ports.comports())
    for p in ports:                      # Espressif USB (ESP32-C3 built-in USB)
        if p.vid == 0x303A:
            return p.device
    for p in ports:
        text = f"{p.description} {p.manufacturer}".lower()
        if any(k in text for k in ("usb serial", "jtag", "ch340", "cp210", "esp")):
            return p.device
    return None


def open_port(name):
    ser = serial.Serial()
    ser.port = name
    ser.baudrate = 115200
    ser.timeout = 0.05
    ser.dtr = False      # avoid resetting the board when the port opens
    ser.rts = False
    ser.open()
    return ser


def handle(line, state):
    parts = line.strip().split(",")
    cmd = parts[0]
    try:
        if cmd == "M" and len(parts) == 3:
            mouse_move(int(parts[1]), int(parts[2]))
        elif cmd == "W" and len(parts) == 2:
            mouse_wheel(int(parts[1]))
        elif cmd == "L" and len(parts) == 1:
            click(LEFTDOWN, LEFTUP)
            print("Left click")
        elif cmd == "R" and len(parts) == 1:
            click(RIGHTDOWN, RIGHTUP)
            print("Right click")
        elif cmd == "S" and len(parts) == 2:
            if parts[1] != state["scroll"]:
                state["scroll"] = parts[1]
                print("Scroll mode:", "ON" if parts[1] == "1" else "OFF")
        elif cmd == "K" and len(parts) == 2:
            if parts[1] != state["lock"]:
                state["lock"] = parts[1]
                print("Cursor lock:", "ON" if parts[1] == "1" else "OFF")
    except ValueError:
        pass


def main():
    print("AeroClick USB receiver. Ctrl+C to quit.")
    print("Close the Arduino Serial Monitor first, or the port will be busy.")
    state = {"scroll": None, "lock": None}

    while True:
        name = find_port()
        if not name:
            print("Waiting for the board (plug in the USB cable)...")
            time.sleep(1.5)
            continue

        try:
            ser = open_port(name)
        except (serial.SerialException, OSError) as e:
            print(f"Cannot open {name}: {e}")
            print("Is the Arduino Serial Monitor still open? Close it and replug.")
            time.sleep(2)
            continue

        print(f"Connected to AeroClick on {name}")
        try:
            while True:
                if ser.in_waiting > 4000:          # fell behind: drop the backlog
                    ser.reset_input_buffer()
                raw = ser.readline()
                if not raw:
                    continue
                handle(raw.decode(errors="ignore"), state)
        except (serial.SerialException, OSError):
            print("Board disconnected. Waiting for it to come back...")
        finally:
            try:
                ser.close()
            except Exception:
                pass
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped")