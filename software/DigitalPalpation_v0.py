#!/usr/bin/env python3
"""
DigitalPalpation_v0.py
======================
Automated needle-insertion + injection routine for the CoreXY (X,Y) + Z stage
+ syringe injector rig, with pressure recording.

  LONGRUNER (motors) : flash DigitalPalpation_motorControls_v0/
  Teensy 4.1 (DAQ)   : flash DigitalPalpation_dataCollections_v0/
                       (NAU7802 24-bit ADC on I2C, SDA 18 / SCL 19)

Workflow
--------
  1. Run:  ./DigitalPalpation_v0.py
           (optional: --motor-port /dev/ttyACM0 --daq-port /dev/ttyACM1, --no-daq)
  2. Welcome screen                                       -> BEGIN
  3. Jog X/Y/Z (and the injector A) to the HOME spot     -> SET HOME
  4. Jog to directly ABOVE sample 1                       -> RECORD POSITION
     ... repeat for samples 2, 3, ... N                  -> COMPLETE SETUP
     (the injector is LOCKED on this screen so nothing gets dispensed or
      air drawn in by accident)
  5. The needle drives back to HOME by itself. READY screen: penetration depth
     (Z steps), injection volume (injector steps), sample rate
     (10/20/40/80/320 SPS), insertion gain and injection gain (1..128). You can
     leave the program here as long as you like and keep jogging - nothing
     moves on its own until RUN TEST.
  6. RUN TEST:
        go to HOME (Z lifts to safe height, XY at speed 3, Z lowers to HOME Z)
        WATER-CAL at HOME  [recorded: injection]
            inject the programmed volume (no insertion), purge 1 s, refill
        for every sample i = 1..N:
            travel to sample i  (safe-Z lift, XY at speed 3, lower to saved Z)
            insert   : Z DOWN by the penetration depth  [recorded: insertion, insertion gain]
            inject   : programmed volume                [recorded: injection, injection gain]
            hold     : RELAX_HOLD_S (5 s), needle stays in [recorded: relaxation]
            withdraw : Z back UP to the saved height above the sample
            travel back to HOME, purge 1 s, refill (injection + purge)
        save CSV (after every sample, so a crash never loses finished samples)
        make the quick-view plots
  STOP RUN (or SPACE) halts every motor immediately and aborts the run.

Motor power: all 4 motors share one enable pin, so they power on/off together.
They power off by themselves after 2 min without motion (Config.MOTOR_IDLE_OFF_S)
to stop them heating up, and power back on automatically for any jog, step
button or test. Step counts are kept while off; just don't push the carriage /
plunger by hand while they're unpowered.

Keyboard: SPACE = stop, ESC = quit, typing numbers on the test-settings screen.
  Moving the motors from the keyboard is currently DISABLED (commented out).
  To bring it back, uncomment every block tagged [KBD-MOTOR]. It was:
  arrows = X/Y   W = WITHDRAW, S = INFUSE   R = Z up, F = Z down   T = jog speed

Data files  (~/digital_palpation/<MM_DD_YYYY>/...)
---------------------------------------------------
  CSV_FILES/DP_<MM_DD_YYYY>_<HHMMSS>.csv            one wide CSV per test
  CSV_FILES/DP_<MM_DD_YYYY>_<HHMMSS>_settings.json  positions, gains, rate, ...
  PNG_FILES/DP_<...>_watercal.png                   injection pressure vs steps
  PNG_FILES/DP_<...>_samples.png                    subplot(2,1,1) insertion,
                                                    subplot(2,1,2) injection,
                                                    sample 1 blue -> last sample red
  CSV columns, left to right (blank cells pad shorter columns):
    WATERCAL_INJECTION-TIME (ms) | WATERCAL_INJECTION-STEPS (steps) | WATERCAL_INJECTION-PRESSURE (bits)
    S1_INSERTION-TIME (ms) | S1_INSERTION-DEPTH (steps) | S1_INSERTION-PRESSURE (bits)
    S1_INJECTION-TIME (ms) | S1_INJECTION-STEPS (steps) | S1_INJECTION-PRESSURE (bits)
    S1_RELAXATION-TIME (ms) | S1_RELAXATION-PRESSURE (bits)
    S2_... (same 8 columns) ... SN_...
  Every phase's TIME restarts at 0 ms when its motor move starts (relaxation:
  0 ms when the injection ends). The PRE_ROLL_S baseline recorded just before
  each move shows up as a few rows with NEGATIVE time and 0 steps; insertion and
  water-cal also keep recording POST_ROLL_S after the move (rows at full depth).

How STEPS are matched to pressure samples (no extra wiring)
-----------------------------------------------------------
  The Teensy timestamps every ADC conversion with its own microsecond clock and
  streams it live. The motor firmware steps on a fixed cadence, so a move of N
  steps takes exactly N x step-delay. The host knows when it told the motor to
  start, so steps(t) = (t - move_start) / step_delay, clamped to 0..N.
  Expected accuracy: about 1-2 steps (USB latency of the motor command).

=============================================================================
SPEED NOTE  (LONGRUNER firmware speed levels -> step delay)
=============================================================================
  Level -> step delay:   1 = 5000 us (200 steps/s)
                         2 = 3000 us (~333 steps/s)
                         3 = 1500 us (~667 steps/s)

  The levels mean different things for different motors:

    Motor                         | LOW / "slow" | HIGH / "fast"
    ------------------------------+--------------+--------------
    CoreXY  X,Y  (Motors 1 & 2)   |      2       |      3
    Injector     (Motor 4, "A")   |      2       |      3
    Z / translation stage (Mot 3) |      1       |      2

  So "move fast" for X, Y, or the injector = speed 3, "slow" = speed 2.
     "move fast" for the stage            = speed 2, "slow" = speed 1.

  AUTOMATED TEST always uses:  CoreXY travel = 3 (fast)
                               Z insertion / withdrawal / lift = 2 (stage fast)
                               Injection, purge, refill = 3 (fast)

  JOGGING has 3 tiers on the speed toggle (see Config.JOG_TIERS):
    GREEN  = FINE   : XY + injector "1.5" (4000 us), Z 7500 us (slower than old level 1)
    ORANGE = MEDIUM : the old LOW  (XY/inj level 2, Z level 1)
    RED    = FAST   : the old HIGH (XY/inj level 3, Z level 2)
=============================================================================

Units: motor positions are raw STEPS, pressure is raw ADC bits (no mm / uL /
kPa calibration yet). Positions are absolute step counts kept by the motor
firmware since the serial port was opened (opening the port resets the
Arduino -> all counters start at 0), so HOME / sample positions are only valid
while this program stays open. The motors are open-loop: if a motor ever
stalls / skips steps, the firmware can't know.
"""
import argparse
import csv
import json
import os
import queue
import struct
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import pygame
import serial
import serial.tools.list_ports


# ============================== CONFIG ======================================
class Config:
    BAUD = 115200
    BOOT_WAIT = 2.2          # s to let the Arduino reset-on-open finish booting
    IDENTIFY_TIMEOUT = 1.5
    MOTOR_ID = b"ID:DP_MOTOR"
    DAQ_ID = b"ID:DP_DAQ"

    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 560
    WINDOW_TITLE = "Digital Palpation v0"
    FPS = 30

    # ---- firmware speed levels (see SPEED NOTE in the header) ----
    STEP_DELAY_US = {1: 5000, 2: 3000, 3: 1500}   # must match stepDelays[] in firmware
    XY_SLOW, XY_FAST = 2, 3
    INJ_SLOW, INJ_FAST = 2, 3
    Z_SLOW, Z_FAST = 1, 2

    RUN_XY_SPEED = XY_FAST     # CoreXY travel during the test (speed 3)
    RUN_Z_SPEED = Z_FAST       # insertions / withdrawals / safe-height lifts (speed 2)
    RUN_INJ_SPEED = INJ_FAST   # injection, purge, refill (speed 3)

    # ---- jog speed tiers (step delays in microseconds; bigger = slower) ----
    #          name      button colour     XY (M1+M2)  Z (M3)  injector (M4)
    JOG_TIERS = [
        ("FINE",   (30, 140, 70),  {"xy": 4000, "z": 7500, "a": 4000}),   # green
        ("MEDIUM", (200, 120, 20), {"xy": 3000, "z": 5000, "a": 3000}),   # orange = old LOW
        ("FAST",   (170, 40, 40),  {"xy": 1500, "z": 3000, "a": 1500}),   # red    = old HIGH
    ]
    JOG_START_TIER = 0

    # ---- directions ----
    Z_DOWN_SIGN = -1           # Motor 3 REVERSE moves the needle DOWN (toward the sample)
    INJ_DISPENSE_SIGN = -1     # Motor 4 REVERSE = INFUSE (pushes liquid out of the syringe)

    # ---- test routine ----
    WATER_CAL_ENABLED = True       # inject at HOME before the first sample
    RELAX_HOLD_S = 5.0             # needle stays in the sample this long after injecting
                                   # (viscous relaxation - recorded at the injection gain)
    PURGE_TIME_S = 1.0             # purge at HOME, run at RUN_INJ_SPEED
    SAFE_Z_EXTRA_LIFT_STEPS = 0    # extra clearance ABOVE the highest saved position for XY travel
    DEFAULT_PENETRATION_STEPS = 200
    DEFAULT_INJECTION_STEPS = 200

    # ---- data acquisition (Teensy + NAU7802) ----
    SAMPLE_RATES = [10, 20, 40, 80, 320]           # SPS choices on the READY screen
    GAINS = [1, 2, 4, 8, 16, 32, 64, 128]          # PGA gain choices
    DEFAULT_RATE = 320
    DEFAULT_GAIN_INSERT = 1
    DEFAULT_GAIN_INJECT = 1
    PRE_ROLL_S = 0.2               # baseline recorded before each move (negative time rows)
    POST_ROLL_S = 0.2              # keep recording this long after insertion / water-cal moves
                                   # finish, so the full depth/volume is always in the data
    MOTOR_CMD_LATENCY_S = 0.002    # host -> LONGRUNER command delay (USB-serial bridge)
    DAQ_CMD_LATENCY_S = 0.0005     # host -> Teensy command delay (native USB)
    DAQ_PACKET = struct.Struct("<HHIiH")           # marker, seq, t_us, reading, crc16
    DAQ_MARKER = 0xAA55

    # ---- motor power (all 4 drivers share ONE enable pin on the CNC shield) ----
    AUTO_MOTOR_OFF = True          # de-energize all motors after MOTOR_IDLE_OFF_S without motion
    MOTOR_IDLE_OFF_S = 120.0       # 2 min. Any jog / step / test re-energizes them automatically.
                                   # Motors always stay energized during a test.

    # ---- link health (for leaving the program open for hours) ----
    HEARTBEAT_S = 2.0          # ping the board if it has been quiet this long
    LINK_TIMEOUT_S = 6.0       # no reply this long -> link flagged as lost

    # ---- files ----
    DATA_DIR = Path.home() / "digital_palpation"     # everything we save lives under here

    @classmethod
    def purge_steps(cls):
        return int(round(cls.PURGE_TIME_S * 1e6 / cls.STEP_DELAY_US[cls.RUN_INJ_SPEED]))

    @classmethod
    def day_dirs(cls, when=None):
        """~/digital_palpation/MM_DD_YYYY/{CSV_FILES,PNG_FILES} (created if needed)."""
        day = cls.DATA_DIR / (when or datetime.now()).strftime("%m_%d_%Y")
        csv_dir, png_dir = day / "CSV_FILES", day / "PNG_FILES"
        csv_dir.mkdir(parents=True, exist_ok=True)
        png_dir.mkdir(parents=True, exist_ok=True)
        return day, csv_dir, png_dir

    @classmethod
    def init_dirs(cls):
        cls.DATA_DIR.mkdir(parents=True, exist_ok=True)


def crc16(data: bytes) -> int:
    """CRC16-CCITT (XModem) - same as the Teensy firmware."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


# ---- CoreXY (copied from PYTHON_MAIN_v0 so the pad matches the real motion) ----
# belt signs (Motor 1, Motor 2) for each compass direction. E/W are mirrored
# vs the textbook convention on this build - see PYTHON_MAIN_v0.py for why.
COREXY_DIRECTIONS = {
    'N': (1, -1), 'S': (-1, 1),
    'E': (-1, -1), 'W': (1, 1),
    'NE': (0, -1), 'SW': (0, 1),
    'NW': (1, 0), 'SE': (-1, 0),
}


def belts_to_xy(b1, b2):
    """Belt step counts -> carriage X/Y in steps, using the table above
    (N = +Y, E = +X).  Display only - positions are stored as belt counts."""
    return (-(b1 + b2)) / 2.0, (b1 - b2) / 2.0


def z_more_up(a, b):
    """Return whichever Z value is higher (further from the sample)."""
    return max(a, b) if Config.Z_DOWN_SIGN < 0 else min(a, b)


# ============================ DEVICE DISCOVERY ==============================
def identify_port(path, cfg):
    """Returns ('motor' | 'daq' | None, note)."""
    try:
        with serial.Serial(path, cfg.BAUD, timeout=0.2) as ser:
            time.sleep(cfg.BOOT_WAIT)
            ser.reset_input_buffer()
            ser.write(b"I\n")
            ser.flush()
            buf = b""
            deadline = time.time() + cfg.IDENTIFY_TIMEOUT
            while time.time() < deadline:
                if ser.in_waiting:
                    buf += ser.read(ser.in_waiting)
                    if cfg.MOTOR_ID in buf:
                        return "motor", None
                    if cfg.DAQ_ID in buf:
                        return "daq", None
                time.sleep(0.05)
            if b"ID:LONGRUNER_MOTOR" in buf:
                return None, "LONGRUNER with OLD firmware - flash DigitalPalpation_motorControls_v0"
            if b"ID:TEENSY_DAQ" in buf:
                return None, "Teensy with OLD firmware - flash DigitalPalpation_dataCollections_v0"
            return None, "no reply"
    except (serial.SerialException, OSError) as e:
        return None, f"could not open ({e})"


def find_ports(cfg, want_daq=True):
    ports = [p for p in serial.tools.list_ports.comports() if p.vid is not None]
    print(f"Scanning {len(ports)} USB serial port(s)...")
    found = {}
    for p in ports:
        role, note = identify_port(p.device, cfg)
        print(f"  {'✓' if role else '?'} {p.device}: {role or note}")
        if role:
            found[role] = p.device
        if "motor" in found and ("daq" in found or not want_daq):
            break
    return found


# ============================== MOTOR LINK ==================================
class Aborted(Exception):
    pass


class MotorLink:
    """Serial connection to DigitalPalpation_motorControls firmware.
    A background reader thread parses every line; a heartbeat thread keeps an
    eye on the link so problems are flagged even while the program idles."""

    def __init__(self, port, cfg):
        self.port, self.cfg = port, cfg
        self.ser = None
        self.pos = [0, 0, 0, 0]
        self.pos_event = threading.Event()
        self.done_event = threading.Event()
        self.done_result = None          # 'DONE' / 'STOPPED' / 'ERR:...'
        self.done_time = 0.0             # host time the last DONE line arrived
        self.enabled = True              # drivers energized? (firmware boots energized)
        self.last_motion = time.time()   # last jog / move command or move completion
        self._wlock = threading.Lock()
        self._running = False
        # health
        self.last_rx = time.time()
        self.fatal_error = None          # serial port gone (USB unplugged etc.)
        self.board_reset = False         # board rebooted -> all positions invalid
        self._booted = threading.Event() # set once the board has identified itself

    # ---- connection ----
    def connect(self):
        print(f"Connecting to motor board on {self.port}")
        self.ser = serial.Serial(self.port, self.cfg.BAUD, timeout=0.1, write_timeout=0.5)
        self._running = True
        self.last_rx = time.time()
        threading.Thread(target=self._reader, daemon=True).start()
        # Opening the port reboots the Arduino. Keep asking "who are you" until
        # it answers, so a slow boot never gets mistaken for a later reset.
        deadline = time.time() + 10.0
        while not self._booted.wait(0.5):
            if time.time() > deadline or self.fatal_error:
                raise RuntimeError("Motor board did not answer after opening the port")
            self.send("I")
        threading.Thread(target=self._heartbeat, daemon=True).start()
        print("✓ Motor board connected")

    def close(self):
        self._running = False
        if self.ser and self.ser.is_open:
            try:
                self.send("X")
                time.sleep(0.1)
            finally:
                try:
                    self.ser.close()
                except Exception:
                    pass

    def send(self, line):
        if self.fatal_error:
            return
        if line[:1] in ("J", "G"):
            self.last_motion = time.time()
            self.enabled = True          # firmware auto-energizes on J / G
        with self._wlock:
            try:
                self.ser.write((line + "\n").encode("ascii"))
            except serial.SerialTimeoutException:
                print(f"Send timed out '{line}'")
            except Exception as e:
                self.fatal_error = f"serial write failed: {e}"
                print(self.fatal_error)

    # ---- health ----
    def link_ok(self):
        return (self.fatal_error is None and not self.board_reset
                and time.time() - self.last_rx < self.cfg.LINK_TIMEOUT_S)

    def health_text(self):
        if self.fatal_error:
            return f"MOTOR LINK LOST ({self.fatal_error}) - restart the program"
        if self.board_reset:
            return "MOTOR BOARD RESET - saved positions are invalid, set HOME again"
        if time.time() - self.last_rx >= self.cfg.LINK_TIMEOUT_S:
            return "NO REPLY FROM MOTOR BOARD - check USB / power"
        return None

    def _heartbeat(self):
        while self._running and not self.fatal_error:
            if time.time() - self.last_rx > self.cfg.HEARTBEAT_S:
                self.send("P")
            time.sleep(0.5)

    # ---- reader thread ----
    def _reader(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception as e:
                if self._running:
                    self.fatal_error = f"serial read failed: {e}"
                    print(self.fatal_error)
                    self.done_result = "LINK LOST"
                    self.done_event.set()
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            now = time.time()
            self.last_rx = now
            if line.startswith("POS ") or line.startswith("DONE "):
                try:
                    self.pos = [int(v) for v in line.split(" ", 1)[1].split(",")]
                except ValueError:
                    continue
                self.pos_event.set()
                if line.startswith("DONE "):
                    self.done_time = now
                    self.last_motion = now
                    self.done_result = "DONE"
                    self.done_event.set()
                continue                       # don't spam the console with POS
            if line == "ID:DP_MOTOR":
                self._booted.set()
            elif line == "READY":              # printed only at boot
                if not self._booted.is_set():
                    continue                   # the expected boot right after opening the port
                self.board_reset = True        # an UNEXPECTED reboot mid-session
                self.pos = [0, 0, 0, 0]
                self.enabled = True            # firmware boots energized
                self.done_result = "BOARD RESET"
                self.done_event.set()
            elif line == "STOPPED":
                self.done_result = "STOPPED"
                self.done_event.set()
            elif line == "ENABLED":
                self.enabled = True
            elif line == "DISABLED":
                self.enabled = False
            elif line.startswith("ERR"):
                self.done_result = line
                self.done_event.set()
            if line not in ("ACK:G",) and not line.startswith(("SPEED:", "DELAY:", "HALT:")):
                print(f"Motor: {line}")

    # ---- commands ----
    def set_speed(self, motor, level):
        self.send(f"S{motor}{int(level)}")

    def set_speeds(self, m1, m2, m3, m4):
        for m, lvl in enumerate((m1, m2, m3, m4), start=1):
            self.set_speed(m, lvl)

    def set_delay(self, motor, us):
        self.send(f"D{motor} {int(us)}")

    def jog(self, motor, sign):
        self.send(f"J{motor}{'+' if sign > 0 else '-'}")

    def halt(self, motor):
        self.send(f"H{motor}")

    def stop_all(self):
        self.send("X")

    def set_enabled(self, on):
        """Energize / de-energize ALL motors (shared enable pin)."""
        self.enabled = bool(on)            # optimistic; confirmed by ENABLED / DISABLED
        if on:
            self.last_motion = time.time()  # fresh idle timer
        self.send("E1" if on else "E0")

    def query_pos(self, timeout=1.0):
        self.pos_event.clear()
        self.send("P")
        if not self.pos_event.wait(timeout):
            raise RuntimeError("No position reply from motor board")
        return list(self.pos)

    def move_abs(self, target, speed_level, abort_event):
        """Absolute move of all 4 motors to `target` (step counts); blocks until
        the firmware reports DONE. Returns (host time G was sent, host time DONE
        arrived). Raises Aborted on STOP / link problems."""
        if not self.link_ok():
            raise Aborted(self.health_text() or "motor link not OK")
        lead = max(abs(t - p) for t, p in zip(target, self.pos))
        if lead == 0:
            now = time.time()
            return now, now
        timeout = lead * self.cfg.STEP_DELAY_US[speed_level] * 1e-6 * 1.5 + 3.0
        self.done_event.clear()
        self.done_result = None
        t_send = time.time()
        self.send("G " + " ".join(str(int(t)) for t in target))
        t_end = time.time() + timeout
        while not self.done_event.wait(0.02):
            if abort_event.is_set():
                raise Aborted("Stopped by user")
            if time.time() > t_end:
                self.stop_all()
                raise Aborted(f"Move timed out after {timeout:.1f}s")
        if abort_event.is_set():
            raise Aborted("Stopped by user")
        if self.done_result != "DONE":
            raise Aborted(f"Move ended with {self.done_result}")
        return t_send, self.done_time


# =============================== DAQ LINK ===================================
class DaqLink:
    """Teensy 4.1 + NAU7802 (DigitalPalpation_dataCollections firmware).
    Text replies and 14-byte CRC'd binary packets share one USB stream; the
    reader thread separates them (packets start with bytes 0x55 0xAA, which
    no ASCII text line can)."""

    def __init__(self, port, cfg):
        self.port, self.cfg = port, cfg
        self.ser = None
        self._running = False
        self._wlock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._lines = queue.Queue()
        self._buf = b""
        self.streaming = False
        self.samples = []                # (t_us, reading) of the current stream
        self.crc_errors = 0
        self.last_rx = time.time()
        self.fatal_error = None
        self.rate = None
        self.gain = None

    def connect(self):
        print(f"Connecting to DAQ (Teensy) on {self.port}")
        self.ser = serial.Serial(self.port, self.cfg.BAUD, timeout=0.05, write_timeout=0.5)
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()
        for _ in range(10):
            try:
                self.command("I", "ID:DP_DAQ", timeout=0.5)
                break
            except RuntimeError:
                continue
        else:
            raise RuntimeError("Teensy did not answer")
        threading.Thread(target=self._heartbeat, daemon=True).start()
        print("✓ DAQ connected")

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                if self.streaming:
                    self._send("X")
                time.sleep(0.05)
            finally:
                self._running = False
                try:
                    self.ser.close()
                except Exception:
                    pass

    # ---- low level ----
    def _send(self, line):
        if self.fatal_error:
            raise RuntimeError(self.fatal_error)
        with self._wlock:
            try:
                self.ser.write((line + "\n").encode("ascii"))
            except Exception as e:
                self.fatal_error = f"DAQ write failed: {e}"
                raise RuntimeError(self.fatal_error)

    def _reader(self):
        P = self.cfg.DAQ_PACKET
        n = P.size
        while self._running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                if self._running:
                    self.fatal_error = f"DAQ read failed: {e}"
                    print(self.fatal_error)
                break
            if not chunk:
                continue
            self.last_rx = time.time()
            buf = self._buf + chunk
            while buf:
                if buf[0] == 0x55:
                    if len(buf) < 2:
                        break
                    if buf[1] == 0xAA:                      # binary packet
                        if len(buf) < n:
                            break
                        marker, seq, t_us, reading, crc = P.unpack_from(buf)
                        if crc16(buf[2:n - 2]) == crc:
                            if self.streaming:
                                self.samples.append((seq, t_us, reading))
                            buf = buf[n:]
                        else:
                            self.crc_errors += 1
                            buf = buf[1:]                   # resync
                        continue
                nl = buf.find(b"\n")
                mk = buf.find(b"\x55\xaa")
                if mk != -1 and (nl == -1 or mk < nl):
                    buf = buf[mk:]                          # junk before a packet
                    continue
                if nl == -1:
                    break
                line = buf[:nl].decode("ascii", errors="ignore").strip()
                buf = buf[nl + 1:]
                if line:
                    if line.startswith("STATUS:"):
                        continue                            # heartbeat reply - just proves it's alive
                    if self._lines.qsize() > 50:            # nobody waiting: never grow unbounded
                        while not self._lines.empty():
                            self._lines.get_nowait()
                    self._lines.put(line)
                    if not line.startswith("ID:"):
                        print(f"DAQ: {line}")
            self._buf = buf

    def command(self, cmd, expect, timeout=3.0):
        """Send a text command and wait for a reply line starting with `expect`."""
        with self._cmd_lock:
            while not self._lines.empty():
                self._lines.get_nowait()
            self._send(cmd)
            end = time.time() + timeout
            while True:
                left = end - time.time()
                if left <= 0:
                    raise RuntimeError(f"DAQ: no '{expect}' reply to '{cmd}'")
                try:
                    line = self._lines.get(timeout=left)
                except queue.Empty:
                    continue
                if line.startswith(expect):
                    return line
                if line.startswith("ERR"):
                    raise RuntimeError(f"DAQ error for '{cmd}': {line}")

    def _heartbeat(self):
        while self._running and not self.fatal_error:
            if (not self.streaming and time.time() - self.last_rx > self.cfg.HEARTBEAT_S
                    and self._cmd_lock.acquire(blocking=False)):
                try:
                    self._send("?")
                except RuntimeError:
                    pass
                finally:
                    self._cmd_lock.release()
            time.sleep(0.5)

    # ---- health ----
    def link_ok(self):
        return self.fatal_error is None and (self.streaming or
                                             time.time() - self.last_rx < self.cfg.LINK_TIMEOUT_S)

    def health_text(self):
        if self.fatal_error:
            return f"DAQ LINK LOST ({self.fatal_error}) - restart the program"
        if not self.link_ok():
            return "NO REPLY FROM TEENSY DAQ - check USB"
        return None

    # ---- acquisition ----
    def set_rate(self, sps):
        if sps != self.rate:
            self.command(f"R{int(sps)}", "RATE:", timeout=3.0)
            self.rate = sps

    def set_gain(self, gain):
        if gain != self.gain:
            self.command(f"G{int(gain)}", "GAIN:", timeout=3.0)
            self.gain = gain

    def start_stream(self):
        """Returns the host time that corresponds to t_us = 0 on the Teensy."""
        self.samples = []
        self.crc_errors = 0
        self.streaming = True
        t_send = time.time()
        try:
            self.command("S", "STREAM_START", timeout=1.0)
        except RuntimeError:
            self.streaming = False
            raise
        return t_send + self.cfg.DAQ_CMD_LATENCY_S

    def stop_stream(self):
        """Returns (samples [(t_us, reading)], info dict)."""
        sent = None
        try:
            reply = self.command("X", "STREAM_STOP", timeout=2.0)
            sent = int(reply.split()[1])
        except (RuntimeError, ValueError, IndexError) as e:
            print(f"DAQ stop: {e}")
        finally:
            self.streaming = False
        got = self.samples
        self.samples = []
        info = {"received": len(got), "sent": sent, "crc_errors": self.crc_errors,
                "dropped": (sent - len(got)) if sent is not None else None}
        return [(t, r) for _, t, r in got], info


# ============================ DATA + FILES ==================================
def split_phase(samples, host_t0, t_move_start, n_steps, step_delay_us, with_relax):
    """Turn a stream into rows.
    samples      : [(t_us, reading)] from the Teensy (t_us = 0 at host_t0)
    t_move_start : host time the motor actually began stepping
    The move ends exactly n_steps * step_delay later (fixed-cadence firmware).
    Returns (move_rows [(time_ms, steps, bits)], relax_rows [(time_ms, bits)])."""
    dur = n_steps * step_delay_us * 1e-6
    t_move_end = t_move_start + dur
    move_rows, relax_rows = [], []
    for t_us, reading in samples:
        ht = host_t0 + t_us * 1e-6
        if with_relax and ht > t_move_end:
            relax_rows.append(((ht - t_move_end) * 1000.0, reading))
        else:
            frac = (ht - t_move_start) / dur if dur > 0 else 1.0
            steps = int(min(max(frac, 0.0), 1.0) * n_steps)   # step k lands at k x delay
            move_rows.append(((ht - t_move_start) * 1000.0, steps, reading))
    return move_rows, relax_rows


def csv_columns(data):
    """Ordered (header, values) list for the wide CSV."""
    cols = []
    wc = data.get("watercal")
    if wc is not None:
        rows = wc["inj"]
        cols += [("WATERCAL_INJECTION-TIME (ms)", [f"{r[0]:.2f}" for r in rows]),
                 ("WATERCAL_INJECTION-STEPS (steps)", [str(r[1]) for r in rows]),
                 ("WATERCAL_INJECTION-PRESSURE (bits)", [str(r[2]) for r in rows])]
    for i in sorted(data.get("samples", {})):
        s = data["samples"][i]
        p = f"S{i}_"
        cols += [(p + "INSERTION-TIME (ms)", [f"{r[0]:.2f}" for r in s["ins"]]),
                 (p + "INSERTION-DEPTH (steps)", [str(r[1]) for r in s["ins"]]),
                 (p + "INSERTION-PRESSURE (bits)", [str(r[2]) for r in s["ins"]]),
                 (p + "INJECTION-TIME (ms)", [f"{r[0]:.2f}" for r in s["inj"]]),
                 (p + "INJECTION-STEPS (steps)", [str(r[1]) for r in s["inj"]]),
                 (p + "INJECTION-PRESSURE (bits)", [str(r[2]) for r in s["inj"]]),
                 (p + "RELAXATION-TIME (ms)", [f"{r[0]:.2f}" for r in s["relax"]]),
                 (p + "RELAXATION-PRESSURE (bits)", [str(r[1]) for r in s["relax"]])]
    return cols


def write_wide_csv(path, data):
    """(Re)write the whole CSV atomically - called after every finished phase."""
    cols = csv_columns(data)
    if not cols:
        return
    nrows = max(len(v) for _, v in cols)
    tmp = path.with_suffix(".csv.tmp")
    with open(tmp, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([h for h, _ in cols])
        for r in range(nrows):
            w.writerow([v[r] if r < len(v) else "" for _, v in cols])
    os.replace(tmp, path)


def make_plots(data, png_dir, stem):
    """Quick-view PNGs. Returns list of saved paths (empty if matplotlib missing)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap, Normalize
        from matplotlib.cm import ScalarMappable
    except ImportError:
        print("⚠ matplotlib not installed - no plots. Install: sudo apt install python3-matplotlib")
        return []

    BLUE, RED = "#1f4fd1", "#d62728"
    ink, grid = "#333333", "#d9d9d9"
    plt.rcParams.update({"axes.edgecolor": "#888888", "axes.labelcolor": ink,
                         "xtick.color": ink, "ytick.color": ink, "axes.titlesize": 12,
                         "axes.spines.top": False, "axes.spines.right": False})
    saved = []

    wc = data.get("watercal")
    if wc and wc["inj"]:
        fig, ax = plt.subplots(figsize=(9, 6.2), dpi=110)
        ax.plot([r[1] for r in wc["inj"]], [r[2] for r in wc["inj"]], color=BLUE, lw=2)
        ax.set_title("Water calibration @ HOME - injection")
        ax.set_xlabel("INJECTION-STEPS (steps)")
        ax.set_ylabel("INJECTION-PRESSURE (bits)")
        ax.grid(True, color=grid, lw=0.8)
        fig.tight_layout()
        p = png_dir / f"{stem}_watercal.png"
        fig.savefig(p)
        plt.close(fig)
        saved.append(p)

    samples = data.get("samples", {})
    if samples:
        ids = sorted(samples)
        n = len(ids)
        cmap = LinearSegmentedColormap.from_list("blue_to_red", [BLUE, RED])
        color = {sid: cmap(k / (n - 1) if n > 1 else 0.0) for k, sid in enumerate(ids)}
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7.6), dpi=110, constrained_layout=True)
        for sid in ids:
            s = samples[sid]
            ax1.plot([r[1] for r in s["ins"]], [r[2] for r in s["ins"]], color=color[sid], lw=1.6)
            ax2.plot([r[1] for r in s["inj"]], [r[2] for r in s["inj"]], color=color[sid], lw=1.6)
        ax1.set_title(f"Insertion - {n} sample(s)")
        ax1.set_xlabel("INSERTION-DEPTH (steps)")
        ax1.set_ylabel("INSERTION-PRESSURE (bits)")
        ax2.set_title("Injection")
        ax2.set_xlabel("INJECTION-STEPS (steps)")
        ax2.set_ylabel("INJECTION-PRESSURE (bits)")
        for ax in (ax1, ax2):
            ax.grid(True, color=grid, lw=0.8)
        # colorbar = the legend: sample 1 (blue) ... sample N (red)
        sm = ScalarMappable(norm=Normalize(vmin=ids[0], vmax=ids[-1] if n > 1 else ids[0] + 1), cmap=cmap)
        cb = fig.colorbar(sm, ax=[ax1, ax2], fraction=0.035, pad=0.01)
        cb.set_label("sample #")
        if n <= 12:
            cb.set_ticks(ids)
        p = png_dir / f"{stem}_samples.png"
        fig.savefig(p)
        plt.close(fig)
        saved.append(p)
    return saved


# =============================== TEST ROUTINE ===============================
class PalpationRun(threading.Thread):
    """Executes the full test in a background thread so the GUI (and its STOP
    button) stays responsive."""

    def __init__(self, link, cfg, home, samples, penetration, injection,
                 daq=None, rate=320, gain_insert=1, gain_inject=1):
        super().__init__(daemon=True)
        self.link, self.cfg, self.daq = link, cfg, daq
        self.home, self.samples = list(home), [list(s) for s in samples]   # [b1, b2, z, a]
        self.pen, self.inj = int(penetration), int(injection)
        self.rate, self.gain_ins, self.gain_inj = rate, gain_insert, gain_inject
        self.purge = cfg.purge_steps()
        self.abort_event = threading.Event()
        self.status = "Starting..."
        self.phase_idx = 0                 # 0 = water-cal, 1..N = samples
        self.n_phases = len(self.samples) + (1 if cfg.WATER_CAL_ENABLED else 0)
        self.done_phases = 0
        self.finished = False
        self.error = None
        self.data = {"watercal": None, "samples": {}}
        self.csv_path = None
        self.png_paths = []
        self.daq_notes = []

        zs = [self.home[2]] + [s[2] for s in self.samples]
        safe = zs[0]
        for z in zs[1:]:
            safe = z_more_up(safe, z)
        self.safe_z = safe - cfg.Z_DOWN_SIGN * cfg.SAFE_Z_EXTRA_LIFT_STEPS

        if daq:
            stamp = datetime.now()
            _, self.csv_dir, self.png_dir = cfg.day_dirs(stamp)
            self.stem = f"DP_{stamp:%m_%d_%Y_%H%M%S}"
            self.csv_path = self.csv_dir / f"{self.stem}.csv"

    def abort(self):
        self.abort_event.set()
        self.link.stop_all()

    # ---- primitive moves (keep the other axes where they are) ----
    def _move(self, b1=None, b2=None, z=None, a=None, speed=None):
        cur = list(self.link.pos)
        tgt = [cur[0] if b1 is None else b1, cur[1] if b2 is None else b2,
               cur[2] if z is None else z, cur[3] if a is None else a]
        return self.link.move_abs(tgt, speed, self.abort_event)

    def _z(self, z):
        return self._move(z=z, speed=self.cfg.RUN_Z_SPEED)

    def _a(self, a):
        return self._move(a=a, speed=self.cfg.RUN_INJ_SPEED)

    def _travel_to(self, p, label):
        """Lift Z to safe height, move XY (straight line), lower Z to p's height.
        Ends EXACTLY at p's X, Y, Z (absolute step counts)."""
        cur_z = self.link.pos[2]
        travel_z = z_more_up(self.safe_z, cur_z)        # never lower before XY travel
        if cur_z != travel_z:
            self.status = f"{label}: lifting to safe Z"
            self._z(travel_z)
        self.status = f"{label}: XY travel"
        self._move(b1=p[0], b2=p[1], speed=self.cfg.RUN_XY_SPEED)
        self.status = f"{label}: lowering to saved Z"
        self._z(p[2])

    def _sleep(self, t):
        end = time.time() + t
        while time.time() < end:
            if self.abort_event.is_set():
                raise Aborted("Stopped by user")
            time.sleep(0.02)

    def _purge_and_refill(self, a_start, tag):
        c = self.cfg
        self.status = f"{tag}: purging {c.PURGE_TIME_S:.1f}s ({self.purge} steps)"
        self._a(a_start + c.INJ_DISPENSE_SIGN * (self.inj + self.purge))
        self.status = f"{tag}: refilling {self.inj + self.purge} steps"
        self._a(a_start)

    def _recorded(self, move_fn, n_steps, speed_level, hold_s, gain, label):
        """Stream pressure around one motor move (+ optional hold).
        Returns (move_rows, relax_rows); with no DAQ just does the motion."""
        c = self.cfg
        if not self.daq:
            move_fn()
            if hold_s > 0:
                self.status = f"{label}: holding {hold_s:.1f}s"
                self._sleep(hold_s)
            return [], []
        if not self.daq.link_ok():
            raise Aborted(self.daq.health_text() or "DAQ link not OK")
        self.status = f"{label}: setting gain {gain}"
        self.daq.set_gain(gain)
        host_t0 = self.daq.start_stream()
        try:
            self._sleep(c.PRE_ROLL_S)
            t_send, _ = move_fn()
            if hold_s > 0:
                self.status = f"{label}: holding {hold_s:.1f}s (relaxation)"
                self._sleep(hold_s)
            else:
                self._sleep(c.POST_ROLL_S)
        finally:
            samples, info = self.daq.stop_stream()
        if info["dropped"] or info["crc_errors"]:
            note = f"{label}: {info['dropped']} dropped, {info['crc_errors']} CRC errors"
            self.daq_notes.append(note)
            print("⚠ " + note)
        delay = c.STEP_DELAY_US[speed_level]
        return split_phase(samples, host_t0, t_send + c.MOTOR_CMD_LATENCY_S,
                           n_steps, delay, with_relax=hold_s > 0)

    def _save(self):
        if self.csv_path:
            try:
                write_wide_csv(self.csv_path, self.data)
            except OSError as e:
                print(f"✗ CSV write failed: {e}")

    def _save_settings(self):
        if not self.csv_path:
            return
        c = self.cfg
        info = {
            "csv": self.csv_path.name, "started": datetime.now().isoformat(timespec="seconds"),
            "home": self.home, "samples": self.samples,
            "penetration_steps": self.pen, "injection_steps": self.inj,
            "sample_rate_sps": self.rate, "gain_insertion": self.gain_ins, "gain_injection": self.gain_inj,
            "relax_hold_s": c.RELAX_HOLD_S, "pre_roll_s": c.PRE_ROLL_S, "post_roll_s": c.POST_ROLL_S,
            "purge_steps": self.purge, "water_cal": c.WATER_CAL_ENABLED,
            "speeds": {"xy": c.RUN_XY_SPEED, "z": c.RUN_Z_SPEED, "injector": c.RUN_INJ_SPEED},
            "note": "positions are [M1 belt, M2 belt, Z, A] step counts for that session",
        }
        (self.csv_dir / f"{self.stem}_settings.json").write_text(json.dumps(info, indent=2))

    def run(self):
        c = self.cfg
        try:
            self.link.set_enabled(True)        # energized for the whole test (holds during relaxation)
            self.link.set_speeds(c.RUN_XY_SPEED, c.RUN_XY_SPEED, c.RUN_Z_SPEED, c.RUN_INJ_SPEED)
            time.sleep(0.05)
            self.link.query_pos()
            if self.daq:
                self.status = f"DAQ: setting {self.rate} SPS"
                self.daq.set_rate(self.rate)
                self._save_settings()

            # ---- 1. always start from HOME ----
            self._travel_to(self.home, "Going HOME")

            # ---- 2. water calibration at HOME (injection only, no insertion) ----
            if c.WATER_CAL_ENABLED:
                tag = "WATER-CAL @ HOME"
                a_start = self.link.pos[3]
                self.status = f"{tag}: injecting {self.inj} steps"
                inj_rows, _ = self._recorded(
                    lambda: self._a(a_start + c.INJ_DISPENSE_SIGN * self.inj),
                    self.inj, c.RUN_INJ_SPEED, 0.0, self.gain_inj, tag)
                if self.daq:
                    self.data["watercal"] = {"inj": inj_rows}
                    self._save()
                self._purge_and_refill(a_start, tag)
                self.done_phases += 1

            # ---- 3. samples ----
            n = len(self.samples)
            for i, p in enumerate(self.samples, start=1):
                self.phase_idx = i
                tag = f"Sample {i}/{n}"
                a_start = self.link.pos[3]

                self._travel_to(p, tag)

                self.status = f"{tag}: inserting {self.pen} steps"
                ins_rows, _ = self._recorded(
                    lambda: self._z(p[2] + c.Z_DOWN_SIGN * self.pen),
                    self.pen, c.RUN_Z_SPEED, 0.0, self.gain_ins, f"{tag} insertion")

                self.status = f"{tag}: injecting {self.inj} steps"
                inj_rows, relax_rows = self._recorded(
                    lambda: self._a(a_start + c.INJ_DISPENSE_SIGN * self.inj),
                    self.inj, c.RUN_INJ_SPEED, c.RELAX_HOLD_S, self.gain_inj, f"{tag} injection")
                if self.daq:
                    self.data["samples"][i] = {"ins": ins_rows, "inj": inj_rows, "relax": relax_rows}
                    self._save()

                self.status = f"{tag}: withdrawing needle"
                self._z(p[2])

                self._travel_to(self.home, f"{tag} -> HOME")
                self._purge_and_refill(a_start, tag)
                self.done_phases += 1

            self.status = (f"Test complete - water-cal + {n} sample(s). Needle is at HOME."
                           if c.WATER_CAL_ENABLED else f"Test complete - {n} sample(s). Needle is at HOME.")
        except Aborted as e:
            self.error = str(e)
            self.status = f"ABORTED: {e}"
            self.link.stop_all()
        except Exception as e:
            self.error = str(e)
            self.status = f"ERROR: {e}"
            self.link.stop_all()
        finally:
            if self.daq and self.daq.streaming:
                try:
                    self.daq.stop_stream()
                except Exception:
                    pass
            if self.csv_path and (self.data["watercal"] or self.data["samples"]):
                final = self.status
                self.status = "Saving CSV + making plots..."
                self._save()
                try:
                    self.png_paths = make_plots(self.data, self.png_dir, self.stem)
                except Exception as e:
                    print(f"✗ Plotting failed: {e}")
                self.status = final + f"  Saved {self.csv_path.name}"
                if self.daq_notes:
                    self.status += "  (DAQ warnings - see terminal)"
            self.finished = True


class ReturnHome(PalpationRun):
    """After COMPLETE SETUP: drive to the saved HOME using the same safe travel
    as a test (Z lifts to the highest saved height, XY at speed 3, Z lowers).
    The injector (A) is not touched."""

    def __init__(self, link, cfg, home, samples):
        super().__init__(link, cfg, home, samples, penetration=1, injection=1, daq=None)

    def run(self):
        c = self.cfg
        try:
            self.link.set_enabled(True)
            self.link.set_speeds(c.RUN_XY_SPEED, c.RUN_XY_SPEED, c.RUN_Z_SPEED, c.RUN_INJ_SPEED)
            time.sleep(0.05)
            self.link.query_pos()
            self._travel_to(self.home, "Returning to HOME")
            self.status = "At HOME"
        except Aborted as e:
            self.error = str(e)
            self.status = f"Stopped: {e}"
            self.link.stop_all()
        except Exception as e:
            self.error = str(e)
            self.status = f"ERROR: {e}"
            self.link.stop_all()
        finally:
            self.finished = True


# ================================ JOGGING ===================================
class Jogger:
    """Hold-to-jog. Each input source (pad button, key, ...) contributes
    {motor: sign}. Active motors get their J command re-sent every 100 ms
    (firmware watchdog is 300 ms); a motor that drops out gets an H."""
    RESEND_S = 0.1

    def __init__(self, link):
        self.link = link
        self.sources = {}
        self.sent = {1: 0, 2: 0, 3: 0, 4: 0}
        self.last_send = 0.0
        self.tier = Config.JOG_START_TIER

    def press(self, source, motor_signs):
        self.sources[source] = motor_signs
        self.update(force=True)

    def release(self, source):
        if self.sources.pop(source, None) is not None:
            self.update(force=True)

    def release_all(self):
        self.sources.clear()
        self.update(force=True)

    def tier_info(self):
        return Config.JOG_TIERS[self.tier]

    def cycle_tier(self):
        self.tier = (self.tier + 1) % len(Config.JOG_TIERS)
        self.apply_speeds()

    def apply_speeds(self):
        d = self.tier_info()[2]
        self.link.set_delay(1, d["xy"])
        self.link.set_delay(2, d["xy"])
        self.link.set_delay(3, d["z"])
        self.link.set_delay(4, d["a"])

    def update(self, force=False):
        desired = {1: 0, 2: 0, 3: 0, 4: 0}
        for signs in self.sources.values():
            desired.update({m: s for m, s in signs.items() if s != 0})
        now = time.time()
        resend = force or (now - self.last_send >= self.RESEND_S)
        for m in desired:
            if desired[m] != 0 and (resend or desired[m] != self.sent[m]):
                self.link.jog(m, desired[m])
            elif desired[m] == 0 and self.sent[m] != 0:
                self.link.halt(m)
        self.sent = desired
        if resend:
            self.last_send = now


# ================================ UI WIDGETS ================================
class Btn:
    def __init__(self, rect, text, color, hover=None, font_size=22, text_color=(255, 255, 255)):
        self.rect = pygame.Rect(rect)
        self.text, self.color = text, color
        self.hover = hover
        self.font = pygame.font.Font(None, font_size)
        self.text_color = text_color
        self.enabled = True
        self.active = False       # e.g. selected field / held jog button

    def hit(self, pos):
        return self.enabled and self.rect.collidepoint(pos)

    def draw(self, screen):
        if not self.enabled:
            col, tcol = (45, 45, 50), (95, 95, 100)
        else:
            over = self.rect.collidepoint(pygame.mouse.get_pos())
            hover = self.hover or tuple(min(255, v + 35) for v in self.color)
            col = hover if (over or self.active) else self.color
            tcol = self.text_color
        pygame.draw.rect(screen, col, self.rect, border_radius=8)
        if self.active:
            pygame.draw.rect(screen, (230, 230, 120), self.rect, 2, border_radius=8)
        lines = self.text.split("\n")
        h = self.font.get_linesize()
        y0 = self.rect.centery - h * len(lines) / 2 + h / 2
        for i, ln in enumerate(lines):
            surf = self.font.render(ln, True, tcol)
            screen.blit(surf, surf.get_rect(center=(self.rect.centerx, y0 + i * h)))


# =================================== UI =====================================
BG = (24, 24, 30)
PANEL = (34, 34, 42)
TXT = (220, 220, 228)
DIM = (130, 130, 145)
GREEN = (40, 120, 70)
RED = (150, 40, 40)
BLUE = (40, 70, 140)
GREY = (70, 70, 78)
TEAL = (30, 100, 110)
SEL = (60, 110, 170)


def fmt_size(n):
    if n < 1024:
        return f"{n} B"
    if n < 1024 ** 2:
        return f"{n / 1024:.1f} KB"
    return f"{n / 1024 ** 2:.1f} MB"


class App:
    # states
    S_WELCOME, S_HOME, S_POS, S_GOHOME, S_PARAMS, S_RUN, S_DONE, S_FILES, S_VIEW = \
        "WELCOME", "HOME", "POSITIONS", "RETURNING HOME", "READY", "RUNNING", "DONE", "FILES", "VIEW"

    WELCOME_TITLE = "Welcome to Digital Palpation!"
    WELCOME_TEXT = ("This system, developed by Brendan Unikewicz, Isaac Dobie, and Tal Cohen "
                    "is designed to help people learn more about soft materials. Before "
                    "performing any testing please be sure to read our GitHub "
                    "to learn more. Thank you!")
    HOME_TEXT = ("Make sure to set the home position of the needle (X, Y, Z, and A). "
                 "Once complete, select SET HOME at the bottom of the screen.")
    POS_TEXT = ("Please select the locations of each test sample. Specifically, place the "
                "needle above each sample you wish to test and record each position by "
                "selecting \"RECORD POSITION\". When complete, move to \"COMPLETE SETUP\".")

    def __init__(self, link, cfg, daq=None):
        self.link, self.cfg, self.daq = link, cfg, daq
        self.jog = Jogger(link)
        pygame.init()
        self.screen = pygame.display.set_mode((cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT))
        pygame.display.set_caption(cfg.WINDOW_TITLE)
        self.f_big = pygame.font.Font(None, 30)
        self.f = pygame.font.Font(None, 22)
        self.f_small = pygame.font.Font(None, 18)
        self.clock = pygame.time.Clock()

        self.state = self.S_WELCOME
        self.prev_state = self.S_WELCOME   # where FILES returns to
        self.home = None
        self.samples = []
        self.fields = {"pen": str(cfg.DEFAULT_PENETRATION_STEPS),
                       "inj": str(cfg.DEFAULT_INJECTION_STEPS)}
        self.active_field = "pen"
        self.rate = cfg.DEFAULT_RATE
        self.gain_ins = cfg.DEFAULT_GAIN_INSERT
        self.gain_inj = cfg.DEFAULT_GAIN_INJECT
        self.run_thread = None
        self.home_thread = None            # automatic return to HOME after COMPLETE SETUP
        self.msg = ""
        self.held_btn = None
        self.handled_reset = False
        # [KBD-MOTOR] keyboard motor-control toggle - disabled for now (see _key)
        # self.keyboard_on = True            # KEYBOARD CONTROL toggle (on by default)
        self.show_keypad = False           # on-screen number pad on the test-settings screen

        # files browser / viewer
        self.files_dir = cfg.DATA_DIR
        self.files_list = []
        self.files_scroll = 0
        self.view_list, self.view_idx, self.view_return = [], 0, self.S_FILES
        self._view_cache = {}

        self._build()
        self.jog.apply_speeds()

    # ---------------------------------------------------------------- layout
    def _build(self):
        # --- left: jog panel ---
        self.b_speed = Btn((20, 78, 150, 34), "", GREY)
        self.b_files = Btn((185, 78, 120, 34), "FILES", TEAL, font_size=24)
        # [KBD-MOTOR] KEYBOARD: ON/OFF toggle button - disabled for now
        # self.b_keys = Btn((320, 78, 150, 34), "", GREY, font_size=20)

        cx, cy, sp, sz = 150, 275, 64, 54
        offs = {'NW': (-1, -1), 'N': (0, -1), 'NE': (1, -1), 'W': (-1, 0),
                'E': (1, 0), 'SW': (-1, 1), 'S': (0, 1), 'SE': (1, 1)}
        self.pad = {d: Btn((cx + dx * sp - sz // 2, cy + dy * sp - sz // 2, sz, sz), d, GREY)
                    for d, (dx, dy) in offs.items()}
        self.pad_center = (cx, cy)

        zd, ad = Config.Z_DOWN_SIGN, Config.INJ_DISPENSE_SIGN
        self.lin = {   # button -> (motor, sign); labels are PHYSICAL directions
            "Z_UP":   (Btn((300, 200, 70, 60), "▲\nUP", BLUE), (3, -zd)),
            "Z_DOWN": (Btn((300, 300, 70, 60), "▼\nDOWN", BLUE), (3, zd)),
            "A_IN":   (Btn((390, 200, 70, 60), "WITHDRAW", (140, 40, 40), font_size=17), (4, -ad)),
            "A_OUT":  (Btn((390, 300, 70, 60), "INFUSE", (140, 40, 40), font_size=19), (4, ad)),
        }

        # --- single-step nudge buttons (CURRENT POSITION panel) ---
        # 2 x 2 grid: X | Y on the first row, Z | A on the second.
        self.nudge = {}       # Btn -> (axis, +1/-1)
        for axis, col, row in (("X", 0, 0), ("Y", 1, 0), ("Z", 0, 1), ("A", 1, 1)):
            bx, by = 148 + col * 222, 454 + row * 32
            self.nudge[Btn((bx, by, 38, 27), "-1", (60, 60, 72), font_size=20)] = (axis, -1)
            self.nudge[Btn((bx + 42, by, 38, 27), "+1", (60, 60, 72), font_size=20)] = (axis, +1)

        # --- right: workflow panel (x 485..790) ---
        R = 495
        self.b_begin = Btn((R, 400, 285, 60), "BEGIN", GREEN, font_size=32)
        self.b_set_home = Btn((R, 400, 285, 60), "SET HOME", GREEN, font_size=30)
        self.b_add = Btn((R, 360, 140, 50), "RECORD\nPOSITION", GREEN)
        self.b_undo = Btn((R + 145, 360, 140, 50), "UNDO LAST", GREY)
        self.b_rehome = Btn((R, 415, 140, 40), "RESET HOME", GREY, font_size=20)
        self.b_save = Btn((R + 145, 415, 140, 40), "COMPLETE\nSETUP", BLUE, font_size=20)

        # READY screen: depth/volume fields, rate + gain selectors, keypad
        self.b_field = {"pen": Btn((R + 165, 52, 120, 30), "", GREY, font_size=26),
                        "inj": Btn((R + 165, 86, 120, 30), "", GREY, font_size=26)}
        self.b_rates = {sps: Btn((R + 92 + i * 39, 124, 36, 28), str(sps), GREY, font_size=20)
                        for i, sps in enumerate(self.cfg.SAMPLE_RATES)}
        self.b_gain = {}       # key -> (minus, plus)
        for key, y in (("ins", 160), ("inj", 194)):
            self.b_gain[key] = (Btn((R + 125, y, 36, 28), "<", GREY, font_size=24),
                                Btn((R + 249, y, 36, 28), ">", GREY, font_size=24))
        self.keypad = {}
        keys = ["7", "8", "9", "4", "5", "6", "1", "2", "3", "CLR", "0", "DEL"]
        for i, k in enumerate(keys):
            r, c = divmod(i, 3)
            self.keypad[k] = Btn((R + 20 + c * 85, 266 + r * 42, 78, 36), k, GREY, font_size=26)
        self.b_keypad = Btn((R, 226, 285, 32), "", GREY, font_size=22)    # show / hide the number pad
        self.b_back = Btn((R, 440, 120, 50), "BACK", GREY)
        self.b_start = Btn((R + 130, 440, 155, 50), "RUN TEST", GREEN, font_size=28)

        self.b_abort = Btn((R, 330, 285, 80), "STOP RUN", RED, font_size=36)
        self.b_stop_home = Btn((R, 330, 285, 80), "STOP", RED, font_size=36)

        # end-of-test screen: just three choices
        self.b_plots = Btn((R, 300, 285, 52), "VIEW PLOTS", TEAL, font_size=28)
        self.b_again = Btn((R, 362, 285, 52), "REPEAT TEST", GREEN, font_size=28)
        self.b_main = Btn((R, 424, 285, 52), "RETURN TO MAIN SCREEN", GREY, font_size=24)

        # --- files screen (full window) ---
        W = self.cfg.WINDOW_WIDTH
        self.b_f_back = Btn((20, 495, 140, 48), "HOME SCREEN", GREY, font_size=24)   # leave FILES
        self.b_f_up = Btn((170, 495, 140, 48), "BACK", GREY, font_size=26)          # up one folder
        self.b_f_refresh = Btn((320, 495, 120, 48), "REFRESH", GREY)
        self.b_f_open = Btn((450, 495, 330, 48), "OPEN IN FILE MANAGER", TEAL)
        self.b_f_scroll_up = Btn((W - 70, 80, 50, 60), "▲", GREY, font_size=28)
        self.b_f_scroll_dn = Btn((W - 70, 420, 50, 60), "▼", GREY, font_size=28)
        self.files_rows_y0, self.files_row_h, self.files_rows = 85, 30, 13

        # --- file viewer (full window) ---
        self.b_v_back = Btn((20, 505, 140, 44), "BACK", GREY, font_size=26)
        self.b_v_prev = Btn((170, 505, 120, 44), "< PREV", GREY)
        self.b_v_next = Btn((300, 505, 120, 44), "NEXT >", GREY)
        self.b_v_open = Btn((430, 505, 350, 44), "OPEN WITH SYSTEM APP", TEAL)

    def _state_buttons(self):
        s = self.state
        if s == self.S_WELCOME:
            return [self.b_begin]
        if s == self.S_HOME:
            return [self.b_set_home]
        if s == self.S_GOHOME:
            return [self.b_stop_home]
        if s == self.S_POS:
            return [self.b_add, self.b_undo, self.b_rehome, self.b_save]
        if s == self.S_PARAMS:
            out = list(self.b_field.values()) + [self.b_keypad, self.b_back, self.b_start]
            if self.show_keypad:
                out += list(self.keypad.values())
            if self.daq:
                out += list(self.b_rates.values()) + [b for pair in self.b_gain.values() for b in pair]
            return out
        if s == self.S_RUN:
            return [self.b_abort]
        if s == self.S_FILES:
            return [self.b_f_back, self.b_f_up, self.b_f_refresh, self.b_f_open,
                    self.b_f_scroll_up, self.b_f_scroll_dn]
        if s == self.S_VIEW:
            return [self.b_v_back, self.b_v_prev, self.b_v_next, self.b_v_open]
        return [self.b_plots, self.b_again, self.b_main]

    def _jog_enabled(self):
        return self.state not in (self.S_RUN, self.S_GOHOME, self.S_FILES, self.S_VIEW)

    def _a_locked(self):
        """Injector (A) can't be moved while picking sample positions."""
        return self.state == self.S_POS

    # ----------------------------------------------------------- actions
    def _record_position(self):
        self.jog.release_all()
        time.sleep(0.05)
        try:
            return self.link.query_pos()
        except RuntimeError as e:
            self.msg = str(e)
            return None

    def _start_run(self):
        if not self.link.link_ok():
            self.msg = "Can't start - motor link problem"
            return
        if self.daq and not self.daq.link_ok():
            self.msg = "Can't start - DAQ (Teensy) link problem"
            return
        try:
            pen, inj = int(self.fields["pen"] or 0), int(self.fields["inj"] or 0)
        except ValueError:
            self.msg = "Depth / volume must be whole numbers"
            return
        if pen <= 0 or inj <= 0:
            self.msg = "Depth and volume must be > 0 steps"
            return
        self.jog.release_all()
        self.run_thread = PalpationRun(self.link, self.cfg, self.home, self.samples, pen, inj,
                                       daq=self.daq, rate=self.rate,
                                       gain_insert=self.gain_ins, gain_inject=self.gain_inj)
        self.run_thread.start()
        self.state = self.S_RUN
        self.msg = ""

    def _click(self, pos):
        if self._jog_enabled():
            if self.b_speed.hit(pos):
                self.jog.cycle_tier()
                return
            if self.b_files.hit(pos):
                self._open_files()
                return
            # [KBD-MOTOR] KEYBOARD toggle click - disabled for now
            # if self.b_keys.hit(pos):
            #     self.keyboard_on = not self.keyboard_on
            #     self.jog.release_all()           # drop anything a key was holding
            #     return
            for d, btn in self.pad.items():
                if btn.hit(pos):
                    b1, b2 = COREXY_DIRECTIONS[d]
                    self._hold(btn, {1: b1, 2: b2})
                    return
            for btn, (m, s) in self.lin.values():
                if btn.hit(pos):
                    if m == 4 and self._a_locked():
                        return
                    self._hold(btn, {m: s})
                    return
            for btn, (axis, d) in self.nudge.items():
                if btn.hit(pos):
                    if axis == "A" and self._a_locked():
                        return
                    self._nudge(axis, d)
                    return
        if self.state == self.S_FILES and self._files_click(pos):
            return
        for b in self._state_buttons():
            if b.hit(pos):
                self._state_action(b)
                return

    # One-step moves, in the same units the CURRENT POSITION readout shows:
    #   X / Y : carriage steps (CoreXY: one X or Y step = one step on BOTH belts)
    #   Z / A : raw motor steps (+1 makes the displayed Z / A number go up by 1)
    NUDGE_BELTS = {"X": (-1, -1), "Y": (1, -1)}     # belt deltas for +1 (matches E / N)

    def _nudge(self, axis, d):
        self.jog.release_all()
        try:
            p = self.link.query_pos(timeout=0.5)      # fresh position, not a stale one
        except RuntimeError as e:
            self.msg = str(e)
            return
        t = list(p)
        if axis in self.NUDGE_BELTS:
            b1, b2 = self.NUDGE_BELTS[axis]
            t[0] += b1 * d
            t[1] += b2 * d
        else:
            t[2 if axis == "Z" else 3] += d
        # Non-blocking: the firmware does this tiny move on its own and the
        # readout updates from its DONE reply. (Busy board just ignores it.)
        self.link.send("G " + " ".join(str(v) for v in t))

    def _hold(self, btn, signs):
        btn.active = True
        self.held_btn = btn
        self.jog.press(("btn", id(btn)), signs)

    def _unclick(self):
        if self.held_btn:
            self.held_btn.active = False
            self.jog.release(("btn", id(self.held_btn)))
            self.held_btn = None

    def _emergency_stop(self):
        self.jog.release_all()
        if self.run_thread and not self.run_thread.finished:
            self.run_thread.abort()
        elif self.home_thread and not self.home_thread.finished:
            self.home_thread.abort()
        else:
            self.link.stop_all()

    def _go_home(self):
        """After COMPLETE SETUP: drive back to the saved HOME (safe-Z lift, XY, lower)."""
        if not self.link.link_ok():
            self.msg = "Can't move - motor link problem"
            return
        self.jog.release_all()
        self.home_thread = ReturnHome(self.link, self.cfg, self.home, self.samples)
        self.home_thread.start()
        self.state = self.S_GOHOME

    def _step_gain(self, key, direction):
        G = self.cfg.GAINS
        cur = self.gain_ins if key == "ins" else self.gain_inj
        new = G[max(0, min(len(G) - 1, G.index(cur) + direction))]
        if key == "ins":
            self.gain_ins = new
        else:
            self.gain_inj = new

    def _state_action(self, b):
        self.msg = ""
        if b is self.b_begin:
            self.state = self.S_HOME
        elif b is self.b_set_home:
            p = self._record_position()
            if p is not None:
                self.home = p
                self.link.board_reset = False     # fresh HOME after a reset is valid again
                self.handled_reset = False
                self.state = self.S_POS
        elif b is self.b_add:
            p = self._record_position()
            if p is not None:
                self.samples.append(p)
        elif b is self.b_undo and self.samples:
            self.samples.pop()
        elif b is self.b_rehome:
            self.home, self.samples = None, []
            self.state = self.S_HOME
        elif b is self.b_save:
            if not self.samples:
                self.msg = "Record at least one sample position first"
            else:
                self._go_home()
        elif b is self.b_stop_home:
            self._emergency_stop()
        elif b in self.b_field.values():
            self.active_field = "pen" if b is self.b_field["pen"] else "inj"
        elif b is self.b_keypad:
            self.show_keypad = not self.show_keypad
        elif b in self.keypad.values():
            self._key_digit(b.text)
        elif b in self.b_rates.values():
            self.rate = int(b.text)
        elif any(b in pair for pair in self.b_gain.values()):
            for key, (minus, plus) in self.b_gain.items():
                if b is minus:
                    self._step_gain(key, -1)
                elif b is plus:
                    self._step_gain(key, +1)
        elif b is self.b_back:
            self.state = self.S_POS
        elif b is self.b_start or b is self.b_again:
            self._start_run()
        elif b is self.b_abort:
            self._emergency_stop()
        elif b is self.b_main:
            # back to the Welcome screen; BEGIN starts a fresh HOME + positions.
            # (depth / volume / rate / gains are remembered for the next setup)
            self.home, self.samples = None, []
            self.state = self.S_WELCOME
        elif b is self.b_plots:
            self._open_viewer(list(self.run_thread.png_paths)[::-1], 0, self.S_DONE)
        # files screen
        elif b is self.b_f_back:
            self.state = self.prev_state
        elif b is self.b_f_up:
            if self.files_dir != self.cfg.DATA_DIR:
                self.files_dir = self.files_dir.parent
                self._refresh_files()
        elif b is self.b_f_refresh:
            self._refresh_files()
        elif b is self.b_f_open:
            self._xdg_open(self.files_dir)
        elif b is self.b_f_scroll_up:
            self.files_scroll = max(0, self.files_scroll - (self.files_rows - 2))
        elif b is self.b_f_scroll_dn:
            max_scroll = max(0, len(self.files_list) - self.files_rows)
            self.files_scroll = min(max_scroll, self.files_scroll + (self.files_rows - 2))
        # viewer
        elif b is self.b_v_back:
            self.state = self.view_return
        elif b is self.b_v_prev and self.view_list:
            self.view_idx = (self.view_idx - 1) % len(self.view_list)
        elif b is self.b_v_next and self.view_list:
            self.view_idx = (self.view_idx + 1) % len(self.view_list)
        elif b is self.b_v_open and self.view_list:
            self._xdg_open(self.view_list[self.view_idx])

    def _xdg_open(self, path):
        try:
            subprocess.Popen(["xdg-open", str(path)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.msg = f"Could not open: {e}"

    def _key_digit(self, k):
        f = self.active_field
        if k == "CLR":
            self.fields[f] = ""
        elif k == "DEL":
            self.fields[f] = self.fields[f][:-1]
        elif k.isdigit() and len(self.fields[f]) < 7:
            self.fields[f] = (self.fields[f] + k).lstrip("0") or "0"

    # ------------------------------------------------------------- files
    def _open_files(self):
        self.jog.release_all()
        self.prev_state = self.state
        self.state = self.S_FILES
        Config.init_dirs()
        self.files_dir = self.cfg.DATA_DIR
        self._refresh_files()

    def _refresh_files(self):
        self.files_scroll = 0
        self._view_cache.clear()
        try:
            entries = [e for e in self.files_dir.iterdir() if not e.name.endswith(".tmp")]
        except OSError as e:
            self.files_list = []
            self.msg = f"Can't read folder: {e}"
            return

        def mtime(p):
            try:
                return p.stat().st_mtime
            except OSError:
                return 0
        dirs = sorted([e for e in entries if e.is_dir()], key=mtime, reverse=True)    # newest day first
        files = sorted([e for e in entries if not e.is_dir()], key=mtime, reverse=True)
        self.files_list = dirs + files

    def _files_click(self, pos):
        """Tap a folder row to open it, or a file row to view it."""
        x, y = pos
        if not (20 <= x <= self.cfg.WINDOW_WIDTH - 85):
            return False
        row = (y - self.files_rows_y0) // self.files_row_h
        if 0 <= row < self.files_rows and y >= self.files_rows_y0:
            idx = self.files_scroll + row
            if idx < len(self.files_list):
                p = self.files_list[idx]
                if p.is_dir():
                    self.files_dir = p
                    self._refresh_files()
                else:
                    files = [f for f in self.files_list if not f.is_dir()]
                    self._open_viewer(files, files.index(p), self.S_FILES)
                return True
        return False

    # ------------------------------------------------------------- viewer
    def _open_viewer(self, paths, idx, return_state):
        if not paths:
            return
        self.view_list, self.view_idx, self.view_return = paths, idx, return_state
        self.state = self.S_VIEW

    def _view_content(self, path):
        """Cached: ('img', surface) | ('text', [lines])."""
        if path in self._view_cache:
            return self._view_cache[path]
        out = ("text", [f"(can't preview {path.name})"])
        try:
            suf = path.suffix.lower()
            if suf in (".png", ".jpg", ".jpeg", ".bmp"):
                img = pygame.image.load(str(path))
                iw, ih = img.get_size()
                bw, bh = self.cfg.WINDOW_WIDTH - 20, 455
                s = min(bw / iw, bh / ih)
                img = pygame.transform.smoothscale(img, (max(1, int(iw * s)), max(1, int(ih * s))))
                out = ("img", img)
            elif suf == ".csv":
                out = ("text", self._csv_summary(path))
            else:
                with open(path, "r", errors="replace") as f:
                    out = ("text", [ln.rstrip("\n")[:95] for ln in f.readlines()[:22]])
        except Exception as e:
            out = ("text", [f"Could not open {path.name}: {e}"])
        self._view_cache[path] = out
        return out

    def _csv_summary(self, path):
        with open(path, newline="") as f:
            rows = list(csv.reader(f))
        if not rows:
            return ["(empty CSV)"]
        header, body = rows[0], rows[1:]
        counts = [sum(1 for r in body if c < len(r) and r[c] != "") for c in range(len(header))]
        groups = {}
        for h, n in zip(header, counts):
            prefix, _, rest = h.partition("_")
            phase = rest.split("-")[0]
            groups.setdefault(prefix, {})
            groups[prefix][phase] = max(groups[prefix].get(phase, 0), n)
        lines = [f"{len(header)} columns, {len(body)} rows", ""]
        for prefix, phases in groups.items():
            lines.append(f"{prefix:<9} " + "  |  ".join(f"{ph.lower()}: {n} pts" for ph, n in phases.items()))
        lines += ["", "Quick-view plots are in ../PNG_FILES  -  'OPEN WITH SYSTEM APP' opens the CSV"]
        return lines[:22]

    # ---------------------------------------------------------- keyboard
    ARROWS = {pygame.K_UP: (0, 1), pygame.K_DOWN: (0, -1),
              pygame.K_LEFT: (-1, 0), pygame.K_RIGHT: (1, 0)}

    def _arrow_dir(self):
        keys = pygame.key.get_pressed()
        dx = sum(v[0] for k, v in self.ARROWS.items() if keys[k])
        dy = sum(v[1] for k, v in self.ARROWS.items() if keys[k])
        return {(0, 1): 'N', (0, -1): 'S', (1, 0): 'E', (-1, 0): 'W',
                (1, 1): 'NE', (-1, 1): 'NW', (1, -1): 'SE', (-1, -1): 'SW'}.get((dx, dy))

    def _key(self, event):
        down = event.type == pygame.KEYDOWN
        k = event.key
        if down and k == pygame.K_SPACE:
            self._emergency_stop()
            return
        if self.state == self.S_VIEW and down and self.view_list:
            if k == pygame.K_LEFT:
                self.view_idx = (self.view_idx - 1) % len(self.view_list)
            elif k == pygame.K_RIGHT:
                self.view_idx = (self.view_idx + 1) % len(self.view_list)
            return
        if self.state == self.S_PARAMS and down:
            if event.unicode.isdigit():
                self._key_digit(event.unicode)
                return
            if k == pygame.K_BACKSPACE:
                self._key_digit("DEL")
                return
            if k == pygame.K_TAB:
                self.active_field = "inj" if self.active_field == "pen" else "pen"
                return
        # ---- [KBD-MOTOR] moving the motors from the keyboard - DISABLED ----
        # To bring it back, uncomment this block AND the other [KBD-MOTOR]
        # blocks (self.keyboard_on in __init__, self.b_keys in _build, its
        # click handler in _click, and its draw lines in _draw).
        #
        # if not self._jog_enabled() or not self.keyboard_on:
        #     return                       # KEYBOARD CONTROL off: keys never move motors
        # if down and k == pygame.K_t:
        #     self.jog.cycle_tier()
        #     return
        # if k in self.ARROWS:
        #     d = self._arrow_dir()
        #     if d:
        #         b1, b2 = COREXY_DIRECTIONS[d]
        #         self.jog.press("arrows", {1: b1, 2: b2})
        #     else:
        #         self.jog.release("arrows")
        #     return
        # # Physical directions: W = WITHDRAW, S = INFUSE (injector),
        # #                      R = Z up,    F = Z down (stage)
        # zd, ad = Config.Z_DOWN_SIGN, Config.INJ_DISPENSE_SIGN
        # keymap = {pygame.K_w: (4, -ad), pygame.K_s: (4, ad),
        #           pygame.K_r: (3, -zd), pygame.K_f: (3, zd)}
        # if k in keymap:
        #     m, s = keymap[k]
        #     if m == 4 and self._a_locked():
        #         return                    # injector locked while picking sample positions
        #     if down:
        #         self.jog.press(("key", k), {m: s})
        #     else:
        #         self.jog.release(("key", k))
        # ---- end [KBD-MOTOR] ----

    # --------------------------------------------------------------- draw
    def _text(self, s, pos, font=None, color=TXT, center=False):
        surf = (font or self.f).render(s, True, color)
        r = surf.get_rect(center=pos) if center else surf.get_rect(topleft=pos)
        self.screen.blit(surf, r)

    def _fmt(self, p):
        x, y = belts_to_xy(p[0], p[1])
        return f"X{x:+.0f} Y{y:+.0f} Z{p[2]:+d}"

    def _draw_files(self):
        sc = self.screen
        W = self.cfg.WINDOW_WIDTH
        self._text("FILES", (20, 14), self.f_big)
        try:
            rel = self.files_dir.relative_to(self.cfg.DATA_DIR.parent)
        except ValueError:
            rel = self.files_dir
        self._text(f"~/{rel}", (110, 18), self.f, DIM)
        self._text("tap a folder to open it, a file to view it  |  newest first", (20, 50), self.f_small, DIM)
        pygame.draw.rect(sc, PANEL, (15, 78, W - 95, self.files_rows * self.files_row_h + 12), border_radius=8)
        shown = self.files_list[self.files_scroll:self.files_scroll + self.files_rows]
        if not shown:
            self._text("(empty)", (30, self.files_rows_y0 + 6), self.f, DIM)
        for i, p in enumerate(shown):
            y = self.files_rows_y0 + i * self.files_row_h
            try:
                st = p.stat()
                when = datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M")
                if p.is_dir():
                    self._text(f"[DIR]  {p.name}/", (30, y + 6), self.f, (150, 200, 240))
                    size = ""
                else:
                    self._text(p.name, (30, y + 6), self.f)
                    size = fmt_size(st.st_size)
                self._text(size, (W - 300, y + 8), self.f_small, DIM)
                self._text(when, (W - 210, y + 8), self.f_small, DIM)
            except OSError:
                self._text(f"{p.name} (unreadable)", (30, y + 6), self.f, DIM)
        total = len(self.files_list)
        if total > self.files_rows:
            self._text(f"{self.files_scroll + 1}-{min(total, self.files_scroll + self.files_rows)} of {total}",
                       (W - 80, 250), self.f_small, DIM)
        self.b_f_up.enabled = self.files_dir != self.cfg.DATA_DIR

    def _draw_viewer(self):
        sc = self.screen
        W = self.cfg.WINDOW_WIDTH
        path = self.view_list[self.view_idx]
        self._text(path.name, (20, 12), self.f, TXT)
        self._text(f"{self.view_idx + 1} / {len(self.view_list)}", (W - 80, 12), self.f, DIM)
        kind, content = self._view_content(path)
        if kind == "img":
            iw, ih = content.get_size()
            sc.blit(content, ((W - iw) // 2, 40 + (455 - ih) // 2))
        else:
            pygame.draw.rect(sc, PANEL, (15, 40, W - 30, 455), border_radius=8)
            for i, ln in enumerate(content):
                self._text(ln, (30, 52 + i * 20), self.f_small, TXT)
        many = len(self.view_list) > 1
        self.b_v_prev.enabled = self.b_v_next.enabled = many

    def _draw(self):
        sc = self.screen
        sc.fill(BG)

        if self.state == self.S_FILES:
            self._draw_files()
        elif self.state == self.S_VIEW:
            self._draw_viewer()
        else:
            pygame.draw.rect(sc, PANEL, (485, 0, 315, self.cfg.WINDOW_HEIGHT))
            self._text("DIGITAL PALPATION v0", (20, 14), self.f_big)
            daq_txt = "DAQ: connected" if self.daq else "DAQ: NOT CONNECTED (motion only, no data)"
            self._text(f"step: {self.state}   |   {daq_txt}", (20, 44), self.f_small,
                       DIM if self.daq else (240, 170, 90))

            # jog panel
            en = self._jog_enabled()
            name, color, d = self.jog.tier_info()
            self.b_speed.text = f"JOG: {name}"
            self.b_speed.color = color
            # [KBD-MOTOR] KEYBOARD toggle button - disabled for now
            # self.b_keys.text = f"KEYBOARD: {'ON' if self.keyboard_on else 'OFF'}"
            # self.b_keys.color = (30, 140, 70) if self.keyboard_on else GREY
            # (and add self.b_keys back into the list just below)
            for b in ([self.b_speed, self.b_files]
                      + list(self.pad.values()) + [v[0] for v in self.lin.values()]):
                b.enabled = en
                if self._a_locked() and (b is self.lin["A_IN"][0] or b is self.lin["A_OUT"][0]):
                    b.enabled = False
                b.draw(sc)
            pygame.draw.circle(sc, (55, 55, 62), self.pad_center, 18)
            self._text("CORE X,Y", (150, 175), self.f_small, DIM, center=True)
            self._text("Z STAGE", (335, 185), self.f_small, DIM, center=True)
            self._text("INJECTOR", (425, 185), self.f_small, DIM, center=True)

            # live position
            p = self.link.pos
            x, y = belts_to_xy(p[0], p[1])
            pygame.draw.rect(sc, PANEL, (15, 425, 455, 115), border_radius=8)
            self._text("CURRENT POSITION (steps)", (28, 435), self.f_small, DIM)
            self._text(f"X {x:+.0f}", (28, 457), self.f_big)
            self._text(f"Y {y:+.0f}", (250, 457), self.f_big)
            self._text(f"Z {p[2]:+d}", (28, 489), self.f_big)
            self._text(f"A {p[3]:+d}", (250, 489), self.f_big)
            for b, (axis, _) in self.nudge.items():
                b.enabled = en and not (axis == "A" and self._a_locked())
                b.draw(sc)
            self._draw_workflow()

        for b in self._state_buttons():
            b.draw(sc)
        if self.msg:
            self._text(self.msg, (498, 505) if self.state not in (self.S_FILES, self.S_VIEW) else (20, 470),
                       self.f_small, (240, 150, 120))

        # link-health banner (always on top)
        warn = self.link.health_text() or (self.daq.health_text() if self.daq else None)
        if warn:
            pygame.draw.rect(sc, (160, 30, 30), (0, 0, self.cfg.WINDOW_WIDTH, 26))
            self._text(warn, (self.cfg.WINDOW_WIDTH // 2, 13), self.f, (255, 255, 255), center=True)
        pygame.display.flip()

    def _draw_workflow(self):
        sc = self.screen
        c = Config
        R = 498
        st = self.state
        if st == self.S_WELCOME:
            y = self._wrap(self.WELCOME_TITLE, (R, 30), 290, font=self.f_big, line_h=30)
            self._wrap(self.WELCOME_TEXT, (R, y + 16), 290, max_lines=14)
        elif st == self.S_HOME:
            self._text("1. Set HOME", (R, 30), self.f_big)
            self._wrap(self.HOME_TEXT, (R, 70), 290)
        elif st == self.S_POS:
            self._text("2. Sample positions", (R, 18), self.f_big)
            y = self._wrap(self.POS_TEXT, (R, 50), 290, font=self.f_small, line_h=17)
            y += 8
            self._text(f"HOME  {self._fmt(self.home)}", (R, y), self.f_small, (150, 210, 150))
            y += 20
            rows = max(1, (330 - y) // 20)
            shown = self.samples[-rows:]
            first = len(self.samples) - len(shown) + 1
            for i, s in enumerate(shown):
                self._text(f"P{first + i:<3d} {self._fmt(s)}", (R, y + i * 20), self.f_small)
            if not self.samples:
                self._text("(no sample positions yet)", (R, y), self.f_small, DIM)
            self._text(f"N = {len(self.samples)}", (R + 200, 335), self.f, TXT)
        elif st == self.S_GOHOME:
            ht = self.home_thread
            self._text("Returning to HOME...", (R, 30), self.f_big, (230, 200, 90))
            if ht:
                self._wrap(ht.status, (R, 70), 285)
        elif st == self.S_PARAMS:
            self._text(f"3. At HOME - {len(self.samples)} samples + water-cal", (R, 18), self.f)
            self._text("Penetration (Z)", (R, 58), self.f)
            self._text("Injection (A)", (R, 92), self.f)
            for k, b in self.b_field.items():
                b.text = self.fields[k] or "0"
                b.active = (k == self.active_field)
            if self.daq:
                self._text("Rate (SPS)", (R, 130), self.f)
                for sps, b in self.b_rates.items():
                    b.color = SEL if sps == self.rate else GREY
                    b.active = sps == self.rate
                for key, y, label, val in (("ins", 160, "Gain insert", self.gain_ins),
                                           ("inj", 194, "Gain inject", self.gain_inj)):
                    self._text(label, (R, y + 6), self.f)
                    pygame.draw.rect(sc, (50, 50, 60), (R + 165, y, 80, 28), border_radius=6)
                    self._text(f"x{val}", (R + 205, y + 14), self.f, TXT, center=True)
                    minus, plus = self.b_gain[key]
                    minus.enabled = val != c.GAINS[0]
                    plus.enabled = val != c.GAINS[-1]
            else:
                self._text("DAQ not connected:", (R, 130), self.f, (240, 170, 90))
                self._text("motion runs, nothing is recorded", (R, 152), self.f_small, DIM)
            self.b_keypad.text = f"DIGITAL KEYBOARD: {'ON' if self.show_keypad else 'OFF'}"
            self.b_keypad.color = (30, 140, 70) if self.show_keypad else GREY
        elif st == self.S_RUN:
            rt = self.run_thread
            self._text("RUNNING", (R, 30), self.f_big, (230, 200, 90))
            if rt.done_phases == 0 and c.WATER_CAL_ENABLED and rt.phase_idx == 0:
                self._text("water-cal at HOME", (R, 66), self.f)
            else:
                self._text(f"sample {rt.phase_idx} of {len(rt.samples)}", (R, 66), self.f)
            self._wrap(rt.status, (R, 100), 285)
            pygame.draw.rect(sc, (60, 60, 70), (R, 290, 285, 14), border_radius=6)
            frac = rt.done_phases / max(1, rt.n_phases)
            pygame.draw.rect(sc, (90, 170, 110), (R, 290, int(285 * frac), 14), border_radius=6)
        elif st == self.S_DONE:
            rt = self.run_thread
            ok = rt and rt.error is None
            self._text("DONE" if ok else "STOPPED", (R, 30), self.f_big,
                       (120, 220, 140) if ok else (230, 110, 110))
            if rt:
                self._wrap(rt.status, (R, 70), 285, max_lines=9)
            # greyed out when the test made no plots (no DAQ / matplotlib / stopped early)
            self.b_plots.enabled = bool(rt and rt.png_paths)

    def _wrap(self, s, pos, width, max_lines=8, font=None, line_h=22, color=TXT):
        """Word-wrap s into width px. Words longer than a line (e.g. URLs) are
        broken after '/' or '-'. Returns the y just below the last line."""
        font = font or self.f
        words = []
        for w in s.split():
            while font.size(w)[0] > width and len(w) > 1:
                cut = len(w)
                while cut > 1 and font.size(w[:cut])[0] > width:
                    cut -= 1
                brk = max(w.rfind("/", 0, cut), w.rfind("-", 0, cut))
                cut = brk + 1 if brk > 0 else cut
                words.append(w[:cut])
                w = w[cut:]
            words.append(w)
        lines, line = [], ""
        for w in words:
            t = (line + " " + w).strip() if line and not line.endswith(("/", "-")) else line + w
            if line and font.size(t)[0] > width:
                lines.append(line)
                line = w
            else:
                line = t
        if line:
            lines.append(line)
        y = pos[1]
        for ln in lines[:max_lines]:
            self._text(ln, (pos[0], y), font, color)
            y += line_h
        return y

    # --------------------------------------------------------------- loop
    def _check_link(self):
        """If the board rebooted (power blip / brown-out), every saved position
        is meaningless - throw them away and send the user back to SET HOME."""
        if self.link.board_reset and not self.handled_reset:
            self.handled_reset = True
            self.jog.release_all()
            self.home, self.samples = None, []
            if self.state not in (self.S_RUN, self.S_FILES, self.S_VIEW):
                self.state = self.S_HOME
            elif self.state in (self.S_FILES, self.S_VIEW):
                self.prev_state = self.S_HOME
                self.view_return = self.S_FILES
            self.jog.apply_speeds()          # board forgot our jog speeds too

    def _auto_motor_off(self):
        """De-energize after MOTOR_IDLE_OFF_S with no motion - never during a test."""
        if not (self.cfg.AUTO_MOTOR_OFF and self.link.enabled and self.link.link_ok()):
            return
        if self.run_thread and not self.run_thread.finished:
            return
        if self.home_thread and not self.home_thread.finished:
            return
        if self.jog.sources:                       # something is being held right now
            return
        if time.time() - self.link.last_motion >= self.cfg.MOTOR_IDLE_OFF_S:
            print(f"Motors idle {self.cfg.MOTOR_IDLE_OFF_S:.0f}s -> de-energized (cooling)")
            self.link.set_enabled(False)

    def run(self):
        running = True
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.type in (pygame.KEYDOWN, pygame.KEYUP):
                    self._key(ev)
                elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                    self._click(ev.pos)
                elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                    self._unclick()
                elif ev.type == getattr(pygame, "WINDOWFOCUSLOST", -1):
                    self._unclick()               # never leave a jog "held"
                    self.jog.release_all()

            self._check_link()
            if self._jog_enabled():
                self.jog.update()
            self._auto_motor_off()
            if self.state == self.S_GOHOME and self.home_thread.finished:
                self.jog.apply_speeds()           # back to jog speeds
                self.state = self.S_PARAMS
                if self.home_thread.error:
                    self.msg = f"Return to HOME stopped ({self.home_thread.error})"
            if self.state == self.S_RUN and self.run_thread.finished:
                self.state = self.S_DONE if self.home else self.S_HOME
                self.jog.apply_speeds()       # back to jog speeds after the test
            self._draw()
            self.clock.tick(self.cfg.FPS)

        self._emergency_stop()
        pygame.quit()


# ================================= MAIN =====================================
def main():
    ap = argparse.ArgumentParser(description="Digital palpation: multi-sample needle insertion + injection")
    ap.add_argument("--motor-port", help="e.g. /dev/ttyACM0 (skips auto-discovery)")
    ap.add_argument("--daq-port", help="Teensy port, e.g. /dev/ttyACM1 (skips auto-discovery)")
    ap.add_argument("--no-daq", action="store_true", help="run motion only, no Teensy / no data")
    args = ap.parse_args()

    cfg = Config()
    Config.init_dirs()
    motor_port, daq_port = args.motor_port, args.daq_port
    want_daq = not args.no_daq
    if not motor_port or (want_daq and not daq_port):
        found = find_ports(cfg, want_daq=want_daq)
        motor_port = motor_port or found.get("motor")
        daq_port = daq_port or found.get("daq")
    if not motor_port:
        print("✗ Motor board not found. Is it flashed with DigitalPalpation_motorControls_v0?")
        print("  Or pass --motor-port /dev/ttyACM0")
        return

    link = MotorLink(port=motor_port, cfg=cfg)
    daq = None
    try:
        link.connect()
        link.query_pos()
        if want_daq:
            if daq_port:
                daq = DaqLink(daq_port, cfg)
                try:
                    daq.connect()
                except Exception as e:
                    print(f"✗ DAQ connect failed: {e} - continuing WITHOUT data recording")
                    daq.close()
                    daq = None
            else:
                print("⚠ Teensy DAQ not found (flash DigitalPalpation_dataCollections_v0?)"
                      " - continuing WITHOUT data recording")
        App(link, cfg, daq).run()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        link.close()
        if daq:
            daq.close()
        print("Motors stopped, ports closed.")


if __name__ == "__main__":
    main()
