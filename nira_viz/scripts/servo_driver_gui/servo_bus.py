"""Serial transport and a single-owner background worker for the command center."""

import math
import queue
import time
from enum import IntEnum

import serial
from PySide6.QtCore import QThread, Signal

ANGLE_MAX = 4095
SPEED_MAX = 3400
ACC_MAX = 150
READ_ATTEMPTS = 3
READ_TIMEOUT = 0.25
WRITE_SETTLE_SECONDS = 0.01
BAUD_RATES = ["1000000", "500000", "250000", "115200"]
BAUD_RATE_CODES = {"1000000": 0, "500000": 1, "250000": 2, "115200": 4}


def command_limits(settings):
    """Use ordinary position limits when available, otherwise the legacy UI range.

    Limit register values alone do not identify the servo's operating mode.
    Keep the raw settings intact, and never rewrite them to normalize the UI.
    """
    minimum, maximum = settings["minimum"], settings["maximum"]
    if 0 <= minimum < maximum <= ANGLE_MAX:
        return minimum, maximum
    return 0, ANGLE_MAX


class Instruction(IntEnum):
    PING = 1
    READ = 2
    WRITE = 3


class Register(IntEnum):
    ID = 5
    BAUD = 6
    MIN_ANGLE = 9
    MAX_ANGLE = 11
    ACC = 41
    GOAL_POSITION = 42
    GOAL_SPEED = 46
    POSITION = 56
    PRESENT_SPEED = 58
    VOLTAGE = 62
    TEMPERATURE = 63
    CURRENT_STATUS = 65


def decode_status(value):
    names = {0: "Voltage", 2: "Overheat", 3: "Stall", 4: "Overcurrent", 5: "Overload"}
    errors = [name for bit, name in names.items() if value & (1 << bit)]
    unknown = value & ~sum(1 << bit for bit in names)
    if unknown:
        errors.append(f"Unknown flags 0x{unknown:02X}")
    return ", ".join(errors) if errors else "Healthy"


class FeetechServo:
    def __init__(self):
        self.ser = None

    def open(self, port, baud):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=0.2)

    def close(self):
        if self.ser:
            self.ser.close()
        self.ser = None

    @staticmethod
    def checksum(data):
        return (~sum(data)) & 0xFF

    def packet(self, sid, inst, params):
        data = [sid, len(params) + 2, inst, *params]
        return bytes([255, 255, *data, self.checksum(data)])

    def write_reg(self, sid, addr, value, size=1):
        params = [int(addr), *int(value).to_bytes(size, "little")]
        self.ser.write(self.packet(sid, Instruction.WRITE, params))
        self.ser.flush()
        # Allow the half-duplex response/turnaround before the next transaction.
        time.sleep(WRITE_SETTLE_SECONDS)

    def _request(self, sid, instruction, params, size, timeout=0.15):
        tx = self.packet(sid, instruction, params)
        self.ser.reset_input_buffer()
        self.ser.write(tx)
        data = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data.extend(self.ser.read(max(1, self.ser.in_waiting)))
            while len(data) >= 6:
                if data[:2] != b"\xff\xff":
                    del data[0]
                    continue
                length = data[3] + 4
                if length < 6 or length > 250:
                    del data[0]
                    continue
                if len(data) < length:
                    break
                frame = bytes(data[:length])
                del data[:length]
                if frame == tx:  # Half-duplex adapters may echo the instruction.
                    continue
                if (frame[2] == sid and frame[3] == size + 2
                        and frame[-1] == self.checksum(frame[2:-1])):
                    return frame[5:-1]
        return None

    def ping(self, sid):
        return self._request(sid, Instruction.PING, [], 0) is not None

    def read_reg(self, sid, addr, size=1):
        for _ in range(READ_ATTEMPTS):
            data = self._request(sid, Instruction.READ, [int(addr), size], size, READ_TIMEOUT)
            if data is not None:
                return data
        return None

    def read_word(self, sid, addr):
        data = self.read_reg(sid, addr, 2)
        return int.from_bytes(data, "little") if data is not None else None

    def snapshot(self, sid):
        data = self.read_reg(sid, Register.POSITION, 10)
        if data is None:
            raise TimeoutError(f"Servo {sid:02d}: feedback read timed out after {READ_ATTEMPTS} attempts")
        command = self.motion_settings(sid)
        speed = int.from_bytes(data[2:4], "little")
        velocity = -(speed & 0x7fff) if speed & 0x8000 else speed
        return dict(position=int.from_bytes(data[:2], "little"), **command,
                    velocity=velocity, voltage=data[6] / 10, temperature=data[7], status=data[9])

    def motion_settings(self, sid):
        data = self.read_reg(sid, Register.ACC, 7)
        if data is None:
            raise TimeoutError(f"Servo {sid:02d}: motion settings read timed out after {READ_ATTEMPTS} attempts")
        return dict(acc=data[0], goal=int.from_bytes(data[1:3], "little"),
                    speed=int.from_bytes(data[5:7], "little"))

    def settings(self, sid):
        data = self.read_reg(sid, Register.MIN_ANGLE, 4)
        if data is None:
            raise TimeoutError(f"Could not read settings for servo {sid:02d}")
        return dict(minimum=int.from_bytes(data[:2], "little"),
                    maximum=int.from_bytes(data[2:], "little"), **self.motion_settings(sid))


class DemoServo:
    """Explicitly simulated hardware; never opens a serial port."""
    def __init__(self):
        self.devices = {sid: dict(position=float(pos), goal=pos, speed=800, acc=50,
                                 minimum=0, maximum=4095, tick=time.monotonic())
                        for sid, pos in [(1, 1536), (2, 2048), (3, 768)]}

    def close(self):
        pass

    def ping(self, sid):
        time.sleep(0.015)
        return sid in self.devices

    def settings(self, sid):
        return self.devices[sid].copy()

    def motion_settings(self, sid):
        return {key: self.devices[sid][key] for key in ("speed", "acc", "goal")}

    def read_word(self, sid, addr):
        if addr == Register.GOAL_POSITION:
            return self.devices[sid]["goal"]
        raise ValueError(f"Unsupported demo register: {addr}")

    def snapshot(self, sid):
        d = self.devices[sid]
        now = time.monotonic()
        delta = d["goal"] - d["position"]
        elapsed = max(1e-6, now - d["tick"])
        step = min(abs(delta), (d["speed"] or 3400) * elapsed)
        d["position"] += math.copysign(step, delta)
        d["tick"] = now
        return dict(position=round(d["position"]), goal=d["goal"], voltage=12.1,
                    temperature=32, status=0, speed=d["speed"], acc=d["acc"],
                    velocity=round(math.copysign(step / elapsed, delta)))

    def write_reg(self, sid, addr, value, size=1):
        if addr == Register.ID:
            self.devices[value] = self.devices.pop(sid)
        elif addr != Register.BAUD:
            key = {Register.GOAL_POSITION: "goal", Register.GOAL_SPEED: "speed",
                   Register.ACC: "acc", Register.MIN_ANGLE: "minimum",
                   Register.MAX_ANGLE: "maximum"}[addr]
            self.devices[sid][key] = value


class BusWorker(QThread):
    connected = Signal(bool, str)
    found = Signal(int)
    scan_progress = Signal(int, int)
    scan_done = Signal()
    selected = Signal(int, dict)
    telemetry = Signal(int, dict)
    failed = Signal(str, str)
    activity = Signal(str)
    changed_id = Signal(int, int)
    completed = Signal(str)
    profile_readback = Signal(int, dict)

    def __init__(self, bus_factory=FeetechServo):
        super().__init__()
        self.commands = queue.Queue()
        self.bus_factory = bus_factory
        self.bus = None
        self.sid = None
        self.cancel_scan = False

    def submit(self, command, *args):
        self.commands.put((command, args))

    def run(self):
        next_poll = 0.0
        while not self.isInterruptionRequested():
            try:
                command, args = self.commands.get(timeout=0.03)
            except queue.Empty:
                command, args = None, ()
            try:
                if command:
                    self._execute(command, args)
            except Exception as exc:
                context = command or "telemetry"
                self.failed.emit(context, str(exc))
                next_poll = time.monotonic() + 0.5
                if command == "connect":
                    self._disconnect()
                elif command == "select":
                    self.sid = None
                elif command == "scan":
                    self.scan_done.emit()
            if self.bus and self.sid is not None and time.monotonic() >= next_poll:
                try:
                    self.telemetry.emit(self.sid, self.bus.snapshot(self.sid))
                    next_poll = time.monotonic() + 0.2
                except Exception as exc:
                    self.failed.emit("telemetry", str(exc))
                    next_poll = time.monotonic() + 0.5
        self._disconnect()

    def _disconnect(self):
        if self.bus:
            self.bus.close()
        self.bus, self.sid = None, None
        self.connected.emit(False, "Disconnected")

    def _execute(self, command, args):
        if command == "connect":
            port, baud, demo = args
            self.bus = DemoServo() if demo else self.bus_factory()
            if not demo:
                self.bus.open(port, baud)
            self.connected.emit(True, "Demo · simulated servos" if demo else f"{port} · {baud:,} baud")
            return
        if command == "disconnect":
            self._disconnect()
            return
        if not self.bus:
            raise RuntimeError("Connect to a bus first")
        if command in {"move", "profile", "hold", "limits", "id", "baud"} and args[0] != self.sid:
            raise RuntimeError("The selected servo changed; select the device again")
        if command == "scan":
            self.sid = None
            start, end = args
            for sid in range(start, end + 1):
                if self.cancel_scan or self.isInterruptionRequested():
                    break
                if self.bus.ping(sid):
                    self.found.emit(sid)
                self.scan_progress.emit(sid - start + 1, end - start + 1)
            self.scan_done.emit()
        elif command == "select":
            self.sid = None
            sid = args[0]
            settings = self.bus.settings(sid)
            if command_limits(settings) != (settings["minimum"], settings["maximum"]):
                self.activity.emit(
                    f"Servo {sid:02d}: raw limit registers {settings['minimum']}–{settings['maximum']}; "
                    "using the legacy 0–4095 command range. Device configuration unchanged.")
            self.sid = sid
            self.selected.emit(sid, settings)
        elif command == "move":
            sid, goal, speed, acc = args
            settings = self.bus.settings(sid)
            minimum, maximum = command_limits(settings)
            if not minimum <= goal <= maximum:
                raise ValueError("Target is outside the current command range / travel limits")
            self.bus.write_reg(sid, Register.ACC, acc)
            self.bus.write_reg(sid, Register.GOAL_SPEED, speed, 2)
            self.bus.write_reg(sid, Register.GOAL_POSITION, goal, 2)
            reported_goal = self.bus.read_word(sid, Register.GOAL_POSITION)
            if reported_goal is None:
                raise TimeoutError("Move was sent, but goal readback timed out after retries. Check live feedback before sending another move.")
            if reported_goal != goal:
                raise RuntimeError("Requested-position readback differs from the target. Check device status and operating mode.")
            self.activity.emit(f"Move sent to servo {sid:02d}: {goal} ticks · speed {speed} · acceleration {acc}")
            self.completed.emit(command)
        elif command == "profile":
            sid, speed, acc = args
            if not 0 <= speed <= SPEED_MAX or not 0 <= acc <= ACC_MAX:
                raise ValueError("Velocity or acceleration is outside the supported range")
            self.bus.write_reg(sid, Register.ACC, acc)
            self.bus.write_reg(sid, Register.GOAL_SPEED, speed, 2)
            try:
                settings = self.bus.motion_settings(sid)
            except TimeoutError as exc:
                raise TimeoutError("Motion profile was sent, but readback timed out. Check the stored values before applying again.") from exc
            self.profile_readback.emit(sid, settings)
            if settings["speed"] != speed or settings["acc"] != acc:
                raise RuntimeError("Motion profile readback differs from the requested values")
            self.activity.emit(f"Servo {sid:02d}: velocity {speed} ticks/s and acceleration {acc} verified")
            self.completed.emit(command)
        elif command == "hold":
            sid = args[0]
            position = self.bus.snapshot(sid)["position"]
            if not 0 <= position <= ANGLE_MAX:
                raise ValueError("Cannot hold: feedback is outside the single-turn range")
            self.bus.write_reg(sid, Register.GOAL_POSITION, position, 2)
            self.activity.emit(f"Hold requested for servo {sid:02d} at {position} ticks")
            self.completed.emit(command)
        elif command == "limits":
            sid, minimum, maximum = args
            self.bus.write_reg(sid, Register.MIN_ANGLE, minimum, 2)
            self.bus.write_reg(sid, Register.MAX_ANGLE, maximum, 2)
            settings = self.bus.settings(sid)
            if settings["minimum"] != minimum or settings["maximum"] != maximum:
                raise RuntimeError("Limit readback differs from the request; check EEPROM write protection")
            self.selected.emit(sid, settings)
            self.activity.emit(f"Servo {sid:02d}: limits verified ({minimum}–{maximum})")
            self.completed.emit(command)
        elif command == "id":
            sid, new_id = args
            if self.bus.ping(new_id):
                raise ValueError(f"ID {new_id} is already in use")
            self.bus.write_reg(sid, Register.ID, new_id)
            time.sleep(0.05)
            if not self.bus.ping(new_id):
                raise RuntimeError("New ID did not respond. Rescan to check the device address.")
            self.sid = new_id
            self.changed_id.emit(sid, new_id)
            self.activity.emit(f"Servo ID changed: {sid:02d} → {new_id:02d}")
            self.completed.emit(command)
        elif command == "baud":
            sid, baud = args
            self.bus.write_reg(sid, Register.BAUD, BAUD_RATE_CODES[baud])
            self.activity.emit(f"Baud change sent to servo {sid:02d}: {baud}. Reconnect at the new rate to verify.")
            self._disconnect()
            self.completed.emit(command)
