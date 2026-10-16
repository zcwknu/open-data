#!/usr/bin/env python3
"""
DigitalPalpation_v01.py
=======================
Automated needle-insertion + injection testing for the CoreXY (X,Y) + Z stage
+ syringe injector rig, with pressure recording.

NEW IN v01 (vs DigitalPalpation_v0.py - same firmware on both boards):
  * The pressure sensor is set up ONCE per test (rate, gain, ADC calibration), left to
    settle for DAQ_WARMUP_S (10 s, motors still), and then streams NON-STOP until the
    test ends. Every recorded phase (water-cal, insertion, injection, relaxation) is cut
    out of that one continuous recording, so the sensor never restarts mid-test.
  * ONE sample rate + ONE gain for the whole test (no separate insertion / injection values).
  * NO zeroing per phase: every kPa value (CSV + plots) is relative to the SETTLING
    PRESSURE = the sensor's mean over the last SETTLE_WINDOW_S of the warm-up. So the
    whole test is on one continuous scale, drift included.
  * Extra figure "Collapsed Pressure Data": each insertion curve starts at 0 kPa (its first
    value) and each injection curve starts at 0 kPa (its first value), so the samples
    lie on top of one another.
  * <trial>_continuous.csv: every sample of the test with what the rig was doing.

  LONGRUNER (motors) : flash DigitalPalpation_motorControls_v0/
  Teensy 4.1 (DAQ)   : flash DigitalPalpation_dataCollections_v0/

Run:  ./DigitalPalpation_v01.py
      (optional: --motor-port /dev/ttyACM0 --daq-port /dev/ttyACM1, --no-daq)

WHERE TO EDIT  (the file reads top -> bottom: settings, logic, look & feel)
-------------
  SETTINGS (class Config, right below)
    SECTION 1  TEST SETTINGS ............. defaults, routine, closed loop, DAQ, homing,
                                           jog speeds - START HERE
    SECTION 2  HARDWARE / COMMUNICATION .. directions, mechanics, microstepping, sensor
                                           -> kPa, encoders; only if the hardware changes
    SECTION 3  ADVANCED TUNING ........... timeouts, encoder analysis, closed-loop
                                           internals, estimate overheads, file names
  PROGRAM LOGIC (no settings in here - everything comes from SECTIONS 1-3)
    SECTION 4  helpers + finding the boards
    SECTION 5  motor board link            SECTION 6  DAQ (Teensy) link
    SECTION 7  data: CSV, kPa, encoders, figures
    SECTION 8  the test routine, re-homing, X-Y homing, jogging
    SECTION 9  start-up (terminal / log file, connecting, opening the window)
  GRAPHIC DESIGN (look & feel only - nothing here changes how a test runs)
    SECTION 10 PLOT STYLE ................ figure colours, sizes, titles, axis labels
    SECTION 11 GUI STYLE ................. window size, fonts, colours, on-screen text,
                                           layout anchors
    SECTION 12 GUI CODE .................. buttons (App._build = the layout) + screens
  REFERENCE NOTES (bottom of the file) ... workflow, CSV format, speed table,
                                           how steps are matched to pressure

Every setting is a CAPITALISED name with a comment saying what it does and its
units. Search the file for the name to see where it is used.
"""
import argparse
import bisect
import collections
import csv
import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading
import time
import warnings
from datetime import datetime, timezone
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")   # no "Hello from the pygame community"
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")          # no sound used: no ALSA chatter
import pygame
import serial
import serial.tools.list_ports


class Config:
    # =========================================================================
    # SECTION 1: TEST SETTINGS  (the things you'd normally change)
    # =========================================================================

    # ---- test parameters: starting values on the test-settings screen (mm, mm/s) ----
    DEFAULT_INSERT_DEPTH_MM = 15.0    # Z: how far the stage pushes the sample onto the needle
    DEFAULT_INSERT_RATE_MM_S = 13.0   # Z: stage speed while inserting
    DEFAULT_RETRACT_DIST_MM = 5.0     # Z: right after inserting, back the needle out this far
                                      # (still recorded, at the insertion SPS/gain); 0 = no retract
    DEFAULT_RETRACT_RATE_MM_S = 13.0  # Z: stage speed while retracting
    DEFAULT_INJECT_DIST_MM = 40.0     # A: plunger travel while injecting
    DEFAULT_INJECT_RATE_MM_S = 3.0    # A: plunger speed while injecting
    # Rates must lie between the quasi-static minimum and the axis maximum, both
    # computed from the mechanics in SECTION 2 (printed in the terminal at start-up).

    # ---- syringes: toggle on the test-settings screen. (name, inner diameter in mm) ----
    # Volume = plunger travel x bore area: nL = mm x pi/4 x ID^2 x 1000
    SYRINGES = [("10 uL", 0.49), ("50 uL", 1.03), ("250 uL", 2.30)]
    DEFAULT_SYRINGE = "10 uL"

    # ---- test routine ----
    WATER_CAL_ENABLED = True       # inject at the FILLING STATION before the first sample
    RELAX_HOLD_S = 5.0             # needle stays in the sample this long after injecting
                                   # (viscous relaxation - recorded at the injection gain)
    PURGE_MM = 2.0                 # quick purge (A infuse) at the filling station, right
                                   # before the plunger is withdrawn to refill the syringe
    SAFE_Z_EXTRA_LIFT_MM = 0.0     # extra stage drop beyond the CUP EXIT / lowest saved position
                                   # before any XY travel (the cup exit itself is set on screen)

    # ---- X-Y re-homing during a test (limit switches = dead-reckoning reset) ----
    REHOME_XY_EACH_SAMPLE = True   # after EVERY sample: stage to safe height, find both X-Y
                                   # limit switches, re-zero, then carry on. Saved positions are
                                   # kept relative to the switch corner, so X-Y drift can't build
                                   # up over long runs. (If X-Y wasn't homed, the test homes first.)

    # ---- closed-loop control of Z and A (needs the Teensy + encoders) ----
    # Set per test on the CLOSED-LOOP screen (after the test settings). Only the RECORDED
    # moves are controlled: insertion (Z) and injection (A). While the move runs, the encoder
    # position is compared with the programmed trajectory every ~CL_PERIOD_S:
    #   LOST STEPS: the motor board's step count vs the encoder -> steps that were lost are
    #   APPENDED to the running move (no stop), so it still ends at the programmed distance.
    #   TIMING: error e = programmed - measured (programmed trajectory timed from the real start
    #   of motion); inside +/- TOL nothing happens; outside, PID on e in units of CL_TAU_S of
    #   programmed travel: speed = programmed x (1 + Kp*en + Ki*int(en)dt/tau + Kd*tau*d(en)/dt),
    #   limited to CL_MIN/MAX_SPEED_FACTOR and the axis maximum.
    #   A last check when stopped makes up any remaining shortfall > TOL, in the SAME direction
    #   (never backwards: the injector never withdraws at a sample).
    # Deviation sign: measured - programmed. NEGATIVE = behind / short of the programmed travel.
    # Recommended starting values (P + feed-forward, a little I, no D):
    #   Z (NEMA23, Tr8x8): tol 0.03 mm - the AS5600's own non-linearity (~ +/-1 deg) is
    #       +/-0.022 mm on an 8 mm/turn screw, so a tighter band just chases sensor error.
    #   A (NEMA11, 1 mm/turn): tol 0.01 mm (~ +/-0.003 mm sensor error, 16 microsteps).
    #   Kp 1.0 -> position loop of 10 rad/s (~0.1 s time constant, ~1.6 Hz). The loop has
    #       ~30-40 ms of delay (25 ms update + USB), which costs only ~20 deg of phase at 10
    #       rad/s - stable with margin. (The "25 Hz servo" rule of thumb online is for an
    #       encoder loop INSIDE the step generator at kHz rates; this loop sits outside it at
    #       40 Hz, so ~1-2 Hz is the right bandwidth here.) Tested in the simulator: 0.5 tracks
    #       ~1.5x worse, 2.0 barely better with less stability margin.
    #   Ki 0.1 -> only trims a slow residual (lost steps are already fed forward).
    #   Kd 0   -> differentiating a 12-bit encoder at 3 ms just amplifies noise.
    #   Leave the axis SPEED HEADROOM: closed loop can't go faster than the axis max
    #   (A at 3.0 of 3.32 mm/s has ~10 %; ~2-2.5 mm/s lets A track to ~0.02 mm).
    CL_DEFAULTS = {   # starting values on the CLOSED-LOOP screen (toggles start OFF)
        "z": {"on": False, "tol": 0.03, "kp": 1.0, "ki": 0.1, "kd": 0.0},
        "a": {"on": False, "tol": 0.01, "kp": 1.0, "ki": 0.1, "kd": 0.0},
    }
    CL_TOL_STEPS_MM = [0.01, 0.02, 0.03, 0.04, 0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5]
    CL_GAIN_STEP = 0.05            # -/+ step for Kp, Ki, Kd (range 0..1)
    CL_PERIOD_S = 0.025            # control update period (also limited by the DAQ sample rate)
    CL_MIN_SPEED_FACTOR = 0.5      # never slower than half / faster than 2x the programmed speed
    CL_MAX_SPEED_FACTOR = 2.0
    CL_INTEGRAL_LIMIT = 4.0        # anti-windup clamp on integral(en dt)
    CL_FINAL_TRIES = 3             # make-up moves after the programmed steps finish
    CL_MAX_CORRECTION = 0.25       # SAFETY: never add more than 25 % of the programmed move. If
                                   # the encoder disagrees by more (slipped magnet, loose wire,
                                   # wrong lead setting) closed loop GIVES UP for that move: the
                                   # move finishes exactly as programmed and it is flagged.
    CL_SETTLE_S = 0.06             # wait this long for fresh encoder samples before checking
    CL_TAU_S = 0.1                 # PID time scale: error is measured in "0.1 s of programmed
                                   # travel"; Kp = 1 -> that much error = +100 % speed, Ki = 1 ->
                                   # it sustained for 0.1 s = +100 %. Tolerance = dead band only.
    CL_MAX_SYNC_S = 0.05           # limit on that calibrated gap
    CL_SYNC_LAG_S = 0.003          # motor board -> host delay of its step-count reports
    CL_SYNC_UNCERTAINTY_S = 0.003  # timing jitter between step count and encoder: lost
                                   # steps are only appended beyond tol + speed x this

    # ---- travel speeds during a test ----
    TRAVEL_JOG_TIER = 0            # moving between positions (XY travel, stage up/down, pulling
                                   # the needle out) uses this JOG tier's speeds: 0 = FINE (slow)
    A_PURGE_MM_S = None            # purge + refill at the filling station (mm/s; None = A max)

    # ---- test record ----
    # TEST_PARAMS_TRIAL_<n>.txt: every parameter of the run + a timestamped timeline, written
    # when the test ends (complete OR stopped). Saved in TRIAL_<n>/ (False = the day's folder)
    # (True = inside TRIAL_<n>/, next to CSV_FILES/ and PNG_FILES/).
    PARAMS_TXT_IN_TRIAL_FOLDER = True

    # ---- time estimate (FINAL CHECK screen + run countdown) ----
    EST_CMD_OVERHEAD_S = 0.03      # per motor move: USB command + DONE reply
    EST_SAVE_PLOTS_S = 3.0         # CSV + plots at the end (DAQ runs only)

    # ---- jogging ----
    # jog speed tiers on the JOG button: FULL-step delays in microseconds (bigger = slower),
    # None = that axis' maximum speed. During sample-position setup the jog is fixed to FINE.
    #          name      button colour     XY (M1+M2)  Z (M3)  injector (M4)
    JOG_TIERS = [
        ("FINE",   (30, 140, 70),  {"xy": 4000, "z": 7500, "a": 4000}),   # green
        ("MEDIUM", (200, 120, 20), {"xy": 3000, "z": 5000, "a": 3000}),   # orange
        ("FAST",   (170, 40, 40),  {"xy": None, "z": None, "a": None}),   # red = axis maximum
    ]
    JOG_START_TIER = 0             # 0 = FINE, 1 = MEDIUM, 2 = FAST
    # size of one -/+ step-button press, in mm (rounded to whole microsteps; see SECTION 2)
    NUDGE_MM = {"x": 0.1, "y": 0.1, "z": 0.1, "a": 0.1}

    # ---- data acquisition (Teensy + NAU7802) ----
    # The sensor is set up ONCE per test, settles for DAQ_WARMUP_S with nothing moving, then
    # streams non-stop until the test ends; each recorded phase is cut out of that stream.
    # So the sample rate and gain are the SAME for the whole test (changing either one
    # re-calibrates the ADC and restarts the settling).
    SAMPLE_RATES = [10, 20, 40, 80, 320]           # SPS choices on the test-settings screen
    GAINS = [1, 2, 4, 8, 16, 32, 64, 128]          # PGA gain choices
    DEFAULT_RATE = 80              # SPS for the whole test (closed loop needs encoder samples:
                                   # use >= 80 SPS with it)
    DEFAULT_GAIN = 8               # gain for the whole test. 8-32 suits this sensor: higher gains
                                   # leave little room next to its ~20 mV offset (clipping)
    DAQ_WARMUP_S = 10.0            # sensor settling time at the START of every test (after the
                                   # ADC calibration, before anything moves)
    SETTLE_WINDOW_S = 1.0          # SETTLING PRESSURE = mean of the last this-many seconds of
                                   # the warm-up (needle in the filling-station cup). Every kPa
                                   # value of the test (CSV + plots) is relative to it - nothing
                                   # else is zeroed. The raw ADC bits are saved untouched.
    COLLAPSE_ZERO_SAMPLES = 1      # "Collapsed Pressure Data" figure: each insertion / injection
                                   # curve minus its first value (1) - or the mean of its first N
    SAVE_CONTINUOUS_CSV = True     # also save every sample of the test: <trial>_continuous.csv
    ADC_RAIL_FRACTION = 0.98       # a reading beyond 98 % of the ADC range = PINNED at its limit
                                   # (open sensor wire, or gain too high). Pinned at the end of the
                                   # warm-up -> the test stops before anything moves.
    PRE_ROLL_S = 0.2               # baseline recorded before each move (negative time rows)
    POST_ROLL_S = 0.2              # keep recording this long after insertion / water-cal moves
                                   # finish, so the full depth/volume is always in the data

    # ---- X-Y homing (limit switches at the SOUTH-WEST corner = -X / -Y) ----
    HOMING_SLOWER_THAN_FINE = 1.1     # ONE constant homing speed: 1.1x slower than the FINE jog (X/Y)
    HOMING_DELAY_US = int(round(JOG_TIERS[0][2]["xy"] * HOMING_SLOWER_THAN_FINE))   # = 4400 us/step
                                      # (bigger = slower). Every homing move uses this speed.
    HOMING_RETREAT_STEPS = 150        # if homing starts ON a switch, move this far off it first
    HOMING_FINAL_BACKOFF_STEPS = 100  # after zeroing at the switch corner, move this far +X and +Y
                                      # so the switches aren't left pressed (0 = stay on them)
    HOMING_MAX_TRAVEL_STEPS = 40000   # give up if a switch isn't found within this many steps

    # ---- X-Y travel limits - enforced ONLY after HOME X-Y (X = 0, Y = 0 at the switches) ----
    # In FULL steps (the units the position readout shows). Jogs stop here, +1 steps and
    # moves past them are refused, and a test won't start with a saved position beyond them.
    XY_MAX_X_STEPS = 725              # X+ limit (measured on the machine)
    XY_MAX_Y_STEPS = 645              # Y+ limit (measured on the machine). None = no limit.

    # ---- CoreXY smoothness: speed ramps (X/Y moves + jogs) ----
    # Moves and jogs start at XY_RAMP_START_US, speed up to the set speed, and slow
    # down again before stopping (releasing a jog slows down instead of stopping dead).
    # Anything at or slower than XY_RAMP_START_US (e.g. the FINE jog) has no ramp at all.
    XY_RAMP_START_US = 4000           # full-step delay at the start/end of a ramp (= FINE speed)
    XY_RAMP_FULLSTEPS = 40            # ramp length (full steps) for a ramp all the way to 0 us;
                                      # FAST (1500 us) ramps over ~25 full steps. 0 = no ramps.

    # ---- motor power (all 4 drivers share ONE enable pin on the CNC shield) ----
    # [MOTOR-POWER] CURRENTLY OFF (2026-10-07): the motors are ALWAYS energized. This
    # board's D8 (driver enable) is shorted to GND, so the firmware is built with
    # ENABLE_PIN_CONTROL 0 and never switches power. All the code is still here.
    # TO BRING IT BACK after a hardware swap: set ENABLE_PIN_CONTROL 1 (+ ENABLE_PIN) in
    # DigitalPalpation_motorControls.cpp, re-flash, then set these two to True.
    MOTOR_POWER_BUTTON = False     # True = POWER: ON/OFF toggle button in the jog panel
    AUTO_MOTOR_OFF = False         # True = de-energize after MOTOR_IDLE_OFF_S without motion
    MOTOR_IDLE_OFF_S = 120.0       # 2 min. Any jog / step / test re-energizes them automatically.
                                   # Motors always stay energized during a test.

    # ---- files ----
    DATA_DIR = Path.home() / "digital_palpation"     # everything we save lives under here

    # ---- terminal output ----
    TERMINAL_VERBOSE = False       # False: the terminal only shows start-up (port scan, board
                                   # connections, encoder/magnet warnings); everything after the
                                   # GUI opens goes to DATA_DIR/logs/DP_log_<MM_DD_YYYY>.txt.
                                   # True: print everything to the terminal as well.

    # =========================================================================
    # SECTION 2: HARDWARE / COMMUNICATION  (only if wiring or firmware changes)
    # =========================================================================

    # ---- motor directions (flip the sign if a motor runs the wrong way) ----
    Z_INSERT_SIGN = +1             # Motor 3 FORWARD raises the Z STAGE (sample/cup) UP toward the
                                   # fixed needle = INSERTS. The opposite (stage DOWN) = clearance.
                                   # The UP jog button moves this way.
    INJ_DISPENSE_SIGN = -1         # Motor 4 REVERSE = INFUSE (pushes liquid out of the syringe)

    # ---- mechanics: every mm <-> step conversion comes from these ----
    FULL_STEPS_PER_REV = 200       # 1.8 degree motors (all four)
    XY_BELT_PITCH_MM = 2.0         # GT2 timing belt
    XY_PULLEY_TEETH = 20           # ASSUMED 20-tooth GT2 pulleys -> 40 mm per turn. CHECK THIS:
                                   # jog X by 50 mm on screen and measure the real travel.
    Z_LEAD_MM = 8.0                # Tr8x8 lead screw: 8 mm of travel per turn
    A_LEAD_MM = 1.0                # NEMA11 mini linear actuator: 1 mm of travel per turn

    # ---- speed limits ----
    MIN_STEP_US = 188              # fastest smooth microstep period of the motor board (~5.3 kHz).
                                   # This sets every axis' maximum ("FAST") speed at 1/8 microstep.
    QUASI_STATIC_FULLSTEPS_S = 3.0 # slowest allowed test rate: 3 full steps per second

    # ---- microstepping: MUST match the M0/M1/M2 jumpers under each driver ----
    # 1/8 = jumpers on M0 + M1 (the first two pin pairs), M2 open - the same on A4988
    # and DRV8825 drivers. No jumpers = full step -> 1.
    # Everything in this file and on screen stays in FULL steps (speeds, depth, volume,
    # purge, limits, homing, the position readout, the CSV DEPTH/STEPS columns); the
    # program converts to microsteps for the firmware. At 1/8 the CSV step columns get
    # 0.125-step resolution.
    XY_MICROSTEPS = 8              # CoreXY belts (X + Y driver slots)
    Z_MICROSTEPS = 8               # Z stage (Z driver slot)
    A_MICROSTEPS = 8               # injector (A driver slot)

    # ---- serial / board identification ----
    BAUD = 115200                  # = SERIAL_BAUD in the motor firmware (Teensy USB ignores it)
    BOOT_WAIT = 2.2                # s to let the Arduino reset-on-open finish booting
    IDENTIFY_TIMEOUT = 1.5         # s to wait for an "ID:" reply while scanning ports
    TEENSY_USB_VID = 0x16C0        # USB vendor ID of every Teensy (PJRC). Used to catch a Teensy
                                   # that was flashed with the MOTOR firmware by mistake.
    MOTOR_ID = b"ID:DP_MOTOR_V4"     # = FIRMWARE_ID in the motor firmware. V4 = + "V us"
                                     # closed-loop speed trim of a running move
    DAQ_ID = b"ID:DP_DAQ_V"          # prefix of FIRMWARE_ID in the Teensy firmware.
                                     # V2 = analog encoders, V3 = I2C (or analog) encoders

    # ---- timing sync + DAQ packet format (must match the Teensy firmware) ----
    MOTOR_CMD_LATENCY_S = 0.002    # host -> LONGRUNER command delay (USB-serial bridge)
    DAQ_CMD_LATENCY_S = 0.0005     # host -> Teensy command delay (native USB)
    DAQ_RX_LATENCY_S = 0.001       # conversion ready on the Teensy -> packet on the host
                                   # (ADC + encoder reads + USB), at its quickest
    DAQ_CLOCK_WINDOW_S = 5.0       # Teensy-clock -> host-clock offset = fastest packet of the
                                   # last this-many seconds (tracks crystal drift during a test)
    DAQ_PACKET = struct.Struct("<HHIiHHH")  # marker, seq, t_us, reading, encA, encZ, crc16
    DAQ_MARKER = 0xAA55

    # ---- pressure sensor -> kPa (NAU7802 24-bit ADC) ----
    # ADC volts = bits / 2^23 x (ADC_FS_FRACTION x ADC_VREF_V / gain)   (NAU7802 full scale
    # is +/-0.5 x VREF / gain; the SparkFun library runs its LDO = VREF at 3.3 V)
    # sensor = SENSOR_MV_PER_V_PER_PSI x SENSOR_EXCITATION_V mV per psi (0.2584 x 3.3 = 0.853)
    NAU7802_WIRING = "3.3V, GND, SDA -> Teensy pin 18, SCL -> pin 19"   # shown in error messages
    ADC_VREF_V = 3.3
    ADC_FS_FRACTION = 0.5
    SENSOR_MV_PER_V_PER_PSI = 0.2584
    SENSOR_EXCITATION_V = 3.3
    KPA_PER_PSI = 6.894757
    PRESSURE_SIGN = +1             # +1 normal. -1 flips every kPa value (CSV kPa columns + plots),
                                   # e.g. if the sensor's S+ / S- wires are swapped at the NAU7802.
                                   # (The raw "bits" columns are never changed.)

    # ---- AS5600 encoders (analog OUT -> Teensy pin 15 = A motor, pin 16 = Z motor) ----
    # Two read modes, chosen in the TEENSY firmware (ENCODER_MODE); the host asks which:
    #   "i2c"    - 12-bit RAW ANGLE over I2C, 0..4095 = one turn (CSV unit: counts).
    #              Z encoder on Wire1 (SDA 17, SCL 16), A encoder on Wire2 (SDA 25, SCL 24).
    #              A failed read arrives as 0xFFFF -> blank in the CSV, skipped in plots.
    #   "analog" - OUT pin -> Teensy pin 15 (A) / 16 (Z), 0 -> 3.3 V = one turn (CSV unit: V)
    ENCODER_VREF = 3.3                # analog: Teensy ADC reference / AS5600 supply (V)
    ENCODER_ADC_MAX = 4095            # analog: Teensy reads them at 12 bits
    ENCODER_I2C_COUNTS = 4096         # i2c: counts per turn
    ENCODER_BAD = 0xFFFF              # i2c: "read failed" marker from the firmware
    # Encoder -> distance for the encoder plot: one shaft turn = one lead.
    ENC_Z_MM_PER_TURN = Z_LEAD_MM     # magnet on the Z motor shaft (Tr8x8 screw)
    ENC_A_MM_PER_TURN = A_LEAD_MM     # magnet on the A motor shaft (1 mm/turn actuator)
    ENC_Z_SIGN = None                 # +1 / -1 if the encoder counts the other way;
    ENC_A_SIGN = None                 # None = pick automatically from the programmed move
    ENC_RATE_WINDOW_S = 0.10          # measured rate = slope fitted over this window
    ENC_PLOT_ALIGN_START = True       # encoder plot: programmed curve starts when the motion
                                      # REALLY starts (removes the ~ms command delay from the
                                      # Deviation row; the delay is printed as the start lag)

    # ---- link health (for leaving the program open for hours) ----
    HEARTBEAT_S = 2.0              # ping the board if it has been quiet this long
    LINK_TIMEOUT_S = 6.0           # no reply this long -> link flagged as lost

    # =========================================================================
    # SECTION 3: ADVANCED TUNING  (internal numbers - the defaults work; change with care)
    # =========================================================================

    # ---- serial link timing ----
    MOTOR_CONNECT_TIMEOUT_S = 10.0   # keep asking the motor board "who are you" this long after
                                     # opening its port (the Arduino reboots when the port opens)
    MOVE_TIMEOUT_FACTOR = 1.5        # a move / homing seek is declared stuck after
    MOVE_TIMEOUT_MARGIN_S = 3.0      # (planned time x FACTOR + MARGIN) -> motors stopped
    JOG_REPEAT_S = 0.1               # a held jog re-sends its command this often; the firmware
                                     # stops a jog it hasn't heard from in JOG_TIMEOUT_MS (300 ms)

    # ---- encoder analysis for the plots (Encoder Data / Encoder Corrected Data) ----
    ENC_START_FRACTION = 0.02        # motion "starts" when the encoder passes 2 % of the move
                                     # (then moved back by the time 2 % takes at the programmed rate)
    ENC_ALIGN_MAX_LAG_S = 0.25       # ENC_PLOT_ALIGN_START only shifts the programmed curve when
                                     # the measured start lag is below this (else: no shift)
    ENC_END_WINDOW_S = 0.15          # "end travel" = mean encoder travel over this long after the
                                     # move ends (printed in the log as "end x / y mm")
    ENC_RATE_FIT_SPAN = (0.1, 0.9)   # "mean rate" = straight-line fit over this part of the move

    # ---- closed-loop internals (see the CL_* settings in SECTION 1) ----
    CL_MOVING_MICROSTEPS = 3         # "it is moving" once the encoder has passed the larger of
    CL_MOVING_ENC_COUNTS = 6         # 3 microsteps or 6 encoder counts (noise floor)
    CL_START_TIMEOUT_S = 0.15        # no movement seen this long -> time from the command instead
    CL_SIGN_LEARN_MM = 0.02          # encoder direction is learned once it has moved this far
    CL_SIGN_LEARN_FRACTION = 0.3     # (or this share of a short move, whichever is smaller)
    CL_RATE_SMOOTHING = 0.3          # weight of each new motor-board step-rate report (0..1)
    CL_MAX_LOST_FRACTION = 0.5       # feed-forward never assumes more than 50 % of steps are lost

    # ---- time estimate overheads (with the EST_* values in SECTION 1) ----
    EST_START_S = 0.2                # enable motors + send speeds at the start of a test
    EST_DAQ_START_S = 1.0            # + set rate / gain / ADC calibration (DAQ runs only;
                                     # the DAQ_WARMUP_S settling is added on top)
    EST_REHOME_OVERHEAD_S = 1.0      # per X-Y re-home: commands, switch settling, zeroing

    # ---- file names ----
    LOG_SUBFOLDER = "logs"           # session logs: DATA_DIR/logs/DP_log_<MM_DD_YYYY>.txt
    FILE_PREFIX = "DP"               # data files: <prefix>_<MM_DD_YYYY>_TRIAL_<n>...

    # ---- derived values / helpers (no need to edit) ----
    @classmethod
    def ms(cls, axis):
        """Microsteps per full step for 'xy', 'z' or 'a'."""
        return {"xy": cls.XY_MICROSTEPS, "z": cls.Z_MICROSTEPS, "a": cls.A_MICROSTEPS}[axis]

    @classmethod
    def step_us(cls, axis, full_step_us):
        """Full-step delay (None = max speed) -> delay per microstep (what the firmware runs)."""
        if full_step_us is None:
            return cls.MIN_STEP_US
        return max(cls.MIN_STEP_US, int(round(full_step_us / cls.ms(axis))))

    # ---- mm <-> steps (axis = 'xy' (carriage X or Y), 'z' or 'a') ----
    @classmethod
    def mm_per_full(cls, axis):
        if axis == "xy":                 # carriage X or Y moves one belt-pitch-tooth per step
            return cls.XY_BELT_PITCH_MM * cls.XY_PULLEY_TEETH / cls.FULL_STEPS_PER_REV
        lead = cls.Z_LEAD_MM if axis == "z" else cls.A_LEAD_MM
        return lead / cls.FULL_STEPS_PER_REV

    @classmethod
    def mm_per_micro(cls, axis):
        return cls.mm_per_full(axis) / cls.ms(axis)

    @classmethod
    def mm_to_micro(cls, axis, mm):
        return int(round(mm / cls.mm_per_micro(axis)))

    @classmethod
    def max_mm_s(cls, axis):
        """FAST: the fastest the motor board steps smoothly (MIN_STEP_US per microstep)."""
        return cls.mm_per_micro(axis) * 1e6 / cls.MIN_STEP_US

    @classmethod
    def min_mm_s(cls, axis):
        """Quasi-static: QUASI_STATIC_FULLSTEPS_S full steps per second."""
        return cls.QUASI_STATIC_FULLSTEPS_S * cls.mm_per_full(axis)

    @classmethod
    def us_for_mm_s(cls, axis, mm_s):
        """Speed in mm/s (None = max) -> microstep delay for the firmware."""
        if mm_s is None:
            return cls.MIN_STEP_US
        return max(cls.MIN_STEP_US, int(round(1e6 * cls.mm_per_micro(axis) / mm_s)))

    @classmethod
    def mm_s_for_us(cls, axis, us):
        return cls.mm_per_micro(axis) * 1e6 / us

    @classmethod
    def travel_delays(cls):
        """{motor: microstep delay} used for moving between positions during a test
        (the TRAVEL_JOG_TIER jog speeds) + the A purge/refill speed."""
        d = cls.JOG_TIERS[cls.TRAVEL_JOG_TIER][2]
        return {1: cls.step_us("xy", d["xy"]), 2: cls.step_us("xy", d["xy"]),
                3: cls.step_us("z", d["z"]), 4: cls.us_for_mm_s("a", cls.A_PURGE_MM_S)}

    # ---- syringe volume ----
    @classmethod
    def syringe_id(cls, name):
        return dict(cls.SYRINGES)[name]

    @classmethod
    def nl_per_mm(cls, name):
        """nL dispensed per mm of plunger travel (bore area in mm^2 x 1000)."""
        d = cls.syringe_id(name)
        return 3.141592653589793 / 4.0 * d * d * 1000.0

    @classmethod
    def speed_table(cls):
        rows = [f"  {'axis':<4} {'mm/full step':>12} {'mm/microstep':>13} {'quasi-static':>13} {'FAST (max)':>11}"]
        for axis, name in (("xy", "X/Y"), ("z", "Z"), ("a", "A")):
            rows.append(f"  {name:<4} {cls.mm_per_full(axis):>12.4f} {cls.mm_per_micro(axis):>13.6f} "
                        f"{cls.min_mm_s(axis):>9.4f} mm/s {cls.max_mm_s(axis):>7.2f} mm/s")
        return "\n".join(rows)

    @classmethod
    def steps(cls, axis, full_steps):
        """Full steps -> microsteps (what the firmware counts)."""
        return int(round(full_steps * cls.ms(axis)))

    @classmethod
    def xy_us(cls, full_step_us):
        return cls.step_us("xy", full_step_us)

    @classmethod
    def xy_steps(cls, full_steps):
        return cls.steps("xy", full_steps)

    @classmethod
    def trial_dirs(cls, when=None):
        """New ~/digital_palpation/MM_DD_YYYY/TRIAL_<n>/{CSV_FILES,PNG_FILES}; n = 1 + the
        highest TRIAL number already in that day's folder. Returns (trial_dir, csv, png)."""
        day = cls.DATA_DIR / (when or datetime.now()).strftime("%m_%d_%Y")
        day.mkdir(parents=True, exist_ok=True)
        n = 1
        for d in day.iterdir():
            if d.is_dir() and d.name.startswith("TRIAL_") and d.name[6:].isdigit():
                n = max(n, int(d.name[6:]) + 1)
        while True:
            trial = day / f"TRIAL_{n}"
            try:
                trial.mkdir()
                break
            except FileExistsError:
                n += 1
        csv_dir, png_dir = trial / "CSV_FILES", trial / "PNG_FILES"
        csv_dir.mkdir()
        png_dir.mkdir()
        return trial, csv_dir, png_dir

    @classmethod
    def init_dirs(cls):
        cls.DATA_DIR.mkdir(parents=True, exist_ok=True)


# ---- CoreXY belt mapping (SECTION 2, hardware) ----
# belt signs (Motor 1, Motor 2) for each compass direction. E/W are mirrored
# vs the textbook convention on this build - see PYTHON_MAIN_v0.py for why.
COREXY_DIRECTIONS = {
    'N': (1, -1), 'S': (-1, 1),
    'E': (-1, -1), 'W': (1, 1),
    'NE': (0, -1), 'SW': (0, 1),
    'NW': (1, 0), 'SE': (-1, 0),
}


# #############################################################################
#                       PROGRAM LOGIC  (how the rig is driven)
# Sections 4-9 talk to the two boards, run the test, and save / plot the data.
# Every number they use comes from SECTIONS 1-3 above; you shouldn't need to
# edit anything here to change a setting.
# #############################################################################

# =============================================================================
# SECTION 4: HELPERS + DEVICE DISCOVERY
# =============================================================================
def crc16(data: bytes) -> int:
    """CRC16-CCITT (XModem) - same as the Teensy firmware."""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def belts_to_xy(b1, b2):
    """Belt counts (microsteps) -> carriage X/Y in FULL steps, using COREXY_DIRECTIONS
    (N = +Y, E = +X). Positions are stored as belt counts; this is for display + limits."""
    ms = Config.XY_MICROSTEPS
    return (-(b1 + b2)) / 2.0 / ms, (b1 - b2) / 2.0 / ms


def z_more_clear(a, b):
    """Return whichever Z (stage) value is further from the needle (stage lower)."""
    return min(a, b) if Config.Z_INSERT_SIGN > 0 else max(a, b)


def fmt_volume(nl):
    """< 1000 nL -> 'xxx nL', otherwise uL (1282 nL -> '1.28 uL')."""
    if abs(nl) < 1000:
        return f"{nl:.1f} nL" if abs(nl) < 100 else f"{nl:.0f} nL"
    return f"{nl / 1000:.2f} uL"


def fmt_flow(nl_s):
    return f"{nl_s:.2f} nL/s" if nl_s < 10 else (f"{nl_s:.1f} nL/s" if nl_s < 100 else f"{nl_s:.0f} nL/s")


def fmt_hms(sec):
    sec = max(0, int(round(sec)))
    return f"{sec // 3600:02d}:{sec % 3600 // 60:02d}:{sec % 60:02d}"



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
            if b"ID:LONGRUNER_MOTOR" in buf or b"ID:DP_MOTOR" in buf:
                return None, "motor board with OLD firmware - re-flash DigitalPalpation_motorControls_v0"
            if b"ID:TEENSY_DAQ" in buf or b"ID:DP_DAQ" in buf:
                return None, "Teensy with OLD firmware - re-flash DigitalPalpation_dataCollections_v0"
            return None, "no reply"
    except (serial.SerialException, OSError) as e:
        return None, f"could not open ({e})"


def find_ports(cfg, want_daq=True):
    """Ask every USB serial port who it is. A role is only accepted from the right HARDWARE:
    a Teensy (USB vendor ID TEENSY_USB_VID) answering as the motor board is running the
    wrong firmware - it is reported and skipped, never used to drive the motors."""
    ports = [p for p in serial.tools.list_ports.comports() if p.vid is not None]
    print(f"Scanning {len(ports)} USB serial port(s)...")
    found = {}
    for p in ports:
        is_teensy = p.vid == cfg.TEENSY_USB_VID
        role, note = identify_port(p.device, cfg)
        board = "Teensy" if is_teensy else "Arduino"
        if role == "motor" and is_teensy:
            print(f"  ✗ {p.device}: this is the TEENSY, but it is running the MOTOR firmware - "
                  "flash DigitalPalpation_dataCollections_v0 onto it (Arduino IDE: Board = Teensy 4.1)")
            continue
        if role and role in found:
            print(f"  ✗ {p.device}: ALSO answers as the {role} board ({found[role]} already did) - two "
                  "boards have the same firmware; re-flash the right sketch onto each")
            continue
        print(f"  {'✓' if role else '?'} {p.device} ({board}): {role or note}")
        if role:
            found[role] = p.device
        if "motor" in found and ("daq" in found or not want_daq):
            break
    return found


# =============================================================================
# SECTION 5: MOTOR BOARD LINK  (LONGRUNER, DigitalPalpation_motorControls firmware)
# =============================================================================
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
        self.limits = (False, False)     # (X switch, Y switch) pressed - from "LIM:" replies
        self.lim_event = threading.Event()
        self.zero_event = threading.Event()
        self.last_motion = time.time()   # last jog / move command or move completion
        self.move_progress = None        # (steps done, steps total, host time) from "VP"
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
        deadline = time.time() + self.cfg.MOTOR_CONNECT_TIMEOUT_S
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
            return "MOTOR BOARD RESET - saved positions are invalid, set them again"
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
            if line.startswith(("POS ", "DONE ", "SEEK:HIT ", "SEEK:MAX ", "SEEK:OTHER ")):
                try:
                    self.pos = [int(v) for v in line.rsplit(" ", 1)[1].split(",")]
                except ValueError:
                    continue
                self.pos_event.set()
                if not line.startswith("POS "):
                    self.done_time = now
                    self.last_motion = now
                    self.done_result = line.split(" ", 1)[0].rstrip(":")   # DONE / SEEK:HIT / SEEK:MAX
                    self.done_event.set()
                continue                       # don't spam the console with POS
            if line.startswith("LIM:"):
                try:
                    x, y = line[4:].split(",")
                    self.limits = (x.strip() == "1", y.strip() == "1")
                except ValueError:
                    pass
                self.lim_event.set()
                continue
            if line == "ZEROED":
                self.zero_event.set()
            if line == self.cfg.MOTOR_ID.decode():
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
            elif line.startswith("VP "):           # closed-loop progress: steps done / total
                try:
                    a, b = line.split()[1:3]
                    self.move_progress = (int(a), int(b), time.time())
                except ValueError:
                    pass
            elif line == "ENABLED":
                self.enabled = True
            elif line == "DISABLED":
                self.enabled = False
            elif line.startswith("ERR"):
                self.done_result = line
                self.done_event.set()
            if line not in ("ACK:G", "ACK:K") and not line.startswith(("SPEED:", "DELAY:", "HALT:", "SOFTMAX:", "RAMP:", "VP ")):
                print(f"Motor: {line}")

    # ---- commands ----
    def set_speed(self, motor, level):
        self.send(f"S{motor}{int(level)}")

    def set_travel_speeds(self):
        """Test travel speeds (SECTION 1: TRAVEL_JOG_TIER + A_PURGE_MM_S) -> microstep delays.
        Returns {motor: delay_us} so callers know exactly what the firmware runs."""
        d = self.cfg.travel_delays()
        for m, us in d.items():
            self.set_delay(m, us)
        return d

    def set_xy_ramp(self):
        """Send the CoreXY speed-ramp settings (scaled to microsteps)."""
        c = self.cfg
        if c.XY_RAMP_FULLSTEPS and c.XY_RAMP_START_US:
            self.send(f"R {c.xy_us(c.XY_RAMP_START_US)} {c.xy_steps(c.XY_RAMP_FULLSTEPS)}")
        else:
            self.send("R 0 0")

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

    def read_limits(self, timeout=1.0):
        """(X pressed, Y pressed) right now."""
        self.lim_event.clear()
        self.send("L")
        if not self.lim_event.wait(timeout):
            raise RuntimeError("No limit-switch reply from motor board")
        return self.limits

    def set_soft_max(self, max_x, max_y):
        """Firmware stops jogs at / refuses moves past these carriage X/Y values
        (None = off). Only meaningful once X-Y is homed."""
        if max_x is None or max_y is None:
            self.send("M0")
        else:                              # limits are in full steps; firmware counts microsteps
            self.send(f"M {self.cfg.xy_steps(max_x)} {self.cfg.xy_steps(max_y)}")

    def zero_belts(self, timeout=1.0):
        """Set the two CoreXY belt counters to 0 (X = 0, Y = 0 here)."""
        self.zero_event.clear()
        self.send("Z12")
        if not self.zero_event.wait(timeout):
            raise RuntimeError("Motor board did not confirm zeroing")
        self.pos[0] = self.pos[1] = 0

    def seek(self, belt_signs, axis, want_pressed, max_steps, delay_us, abort_event):
        """Step the belts in belt_signs until the axis ('X'/'Y') switch reads
        want_pressed (firmware stops within ~1 step). Returns True on HIT,
        False if max_steps ran out. Raises Aborted on STOP / link problems."""
        if not self.link_ok():
            raise Aborted(self.health_text() or "motor link not OK")
        s1, s2 = belt_signs
        self.done_event.clear()
        self.done_result = None
        self.send(f"K {int(s1)} {int(s2)} {axis} {1 if want_pressed else 0} {int(max_steps)}")
        c = self.cfg
        t_end = time.time() + max_steps * delay_us * 1e-6 * c.MOVE_TIMEOUT_FACTOR + c.MOVE_TIMEOUT_MARGIN_S
        while not self.done_event.wait(0.02):
            if abort_event.is_set():
                raise Aborted("Stopped by user")
            if time.time() > t_end:
                self.stop_all()
                raise Aborted("Homing move timed out")
        if abort_event.is_set():
            raise Aborted("Stopped by user")
        if self.done_result == "SEEK:OTHER":
            other = "Y" if axis == "X" else "X"
            raise Aborted(f"moving toward the {axis} switch closed the {other} switch - "
                          "are the X and Y switch wires swapped on the shield?")
        if self.done_result not in ("SEEK:HIT", "SEEK:MAX"):
            raise Aborted(f"Homing move ended with {self.done_result}")
        return self.done_result == "SEEK:HIT"

    def move_abs(self, target, step_us, abort_event, on_tick=None, slow_factor=1.0):
        """Absolute move of all 4 motors to `target` (step counts); blocks until
        the firmware reports DONE. Returns (host time G was sent, host time DONE
        arrived). Raises Aborted on STOP / link problems.
        on_tick(): called every ~20 ms while the move runs (closed-loop control);
        slow_factor: how much slower than step_us the move may end up (timeout)."""
        if not self.link_ok():
            raise Aborted(self.health_text() or "motor link not OK")
        lead = max(abs(t - p) for t, p in zip(target, self.pos))
        if lead == 0:
            now = time.time()
            return now, now
        c = self.cfg                                   # step_us = microstep delay in use
        timeout = lead * step_us * 1e-6 * c.MOVE_TIMEOUT_FACTOR * slow_factor + c.MOVE_TIMEOUT_MARGIN_S
        self.done_event.clear()
        self.done_result = None
        t_send = time.time()
        self.send("G " + " ".join(str(int(t)) for t in target))
        t_end = time.time() + timeout
        while not self.done_event.wait(0.02):
            if on_tick:
                on_tick()
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


# =============================================================================
# SECTION 6: DAQ LINK  (Teensy 4.1 + NAU7802, DigitalPalpation_dataCollections firmware)
# =============================================================================
class DaqLink:
    """Teensy 4.1 + NAU7802 (DigitalPalpation_dataCollections firmware).
    Text replies and 14-byte CRC'd binary packets share one USB stream; the
    reader thread separates them (packets start with bytes 0x55 0xAA, which
    no ASCII text line can)."""

    def __init__(self, port, cfg):
        self.port, self.cfg = port, cfg
        self.ser = None
        self._running = False
        self.enc_mode = "analog"         # "i2c" once the V3 firmware says so (connect)
        self.last_status = ""            # latest "STATUS:..." line
        self._status_event = threading.Event()
        self._wlock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._lines = queue.Queue()
        self._buf = b""
        self.streaming = False
        self.samples = []                # (seq, t_us, reading, encA, encZ) since the last drain()
        self._slock = threading.Lock()   # guards swapping `samples` (drain) vs appending (reader)
        self.n_received = 0              # packets received in the current stream
        self.crc_errors = 0
        self._t_wrap, self._t_last = 0, None   # t_us unwrapping (Teensy's uint32 wraps at ~71.6 min)
        self._clock = collections.deque()      # sliding minimum of (rx time - t_us), see clock_offset
        self.offset_now = None           # host time of t_us = 0, tracked during the stream
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
                self.command("I", self.cfg.DAQ_ID.decode(), timeout=0.5)
                break
            except RuntimeError:
                continue
        else:
            raise RuntimeError("Teensy did not answer")
        self.enc_mode = "analog"                 # V2 firmware: analog encoders only
        try:
            self._status_event.clear()
            self._send("?")
            self._status_event.wait(1.5)
            if "scale=0" in self.last_status:
                print(f"⚠ NAU7802 pressure ADC not found by the Teensy - check its wiring: "
                      f"{self.cfg.NAU7802_WIRING}. Tests can't start until it is found.")
            if "enc=i2c" in self.last_status:
                self.enc_mode = "i2c"
                mag = self.command("M", "MAG:", timeout=1.0)     # magnet + I2C link check
                print(f"DAQ: {mag}")
                if "MISSING" in mag:
                    print("⚠ AS5600 encoder not answering on I2C (see MAG line) - check its wiring")
                elif "md=0" in mag or "ml=1" in mag or "mh=1" in mag:
                    print("⚠ AS5600 magnet problem (md=0 no magnet, ml=1 too weak/far, mh=1 too strong/close)")
        except RuntimeError:
            pass
        threading.Thread(target=self._heartbeat, daemon=True).start()
        print(f"✓ DAQ connected (encoders: {self.enc_mode})")

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
            rx = self.last_rx = time.time()
            buf = self._buf + chunk
            while buf:
                if buf[0] == 0x55:
                    if len(buf) < 2:
                        break
                    if buf[1] == 0xAA:                      # binary packet
                        if len(buf) < n:
                            break
                        marker, seq, t_us, reading, enc_a, enc_z, crc = P.unpack_from(buf)
                        if crc16(buf[2:n - 2]) == crc:
                            if self.streaming:
                                self._add_sample(seq, t_us, reading, enc_a, enc_z, rx)
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
                        self.last_status = line             # heartbeat reply - proves it's alive
                        self._status_event.set()
                        continue
                    if self._lines.qsize() > 50:            # nobody waiting: never grow unbounded
                        while not self._lines.empty():
                            self._lines.get_nowait()
                    self._lines.put(line)
                    if not line.startswith(("ID:", "MAG:")):     # MAG: printed once by connect()
                        print(f"DAQ: {line}")
            self._buf = buf

    def _add_sample(self, seq, t_us, reading, enc_a, enc_z, rx):
        """One packet of the running stream: unwrap the Teensy's 32-bit microsecond clock,
        track its offset to the host clock, store the sample."""
        t = t_us + self._t_wrap
        if self._t_last is not None and t < self._t_last - (1 << 31):
            self._t_wrap += 1 << 32
            t += 1 << 32
        self._t_last = t
        # host time of t_us = 0: every packet arrives AFTER its conversion, so the smallest
        # (arrival - t_us) over a short window is the truest; the window follows the drift
        # between the two crystals (~0.1-0.2 s per hour) during a long test.
        d = rx - t * 1e-6 - self.cfg.DAQ_RX_LATENCY_S
        q = self._clock
        while q and q[-1][1] >= d:
            q.pop()
        q.append((t * 1e-6, d))
        while q[0][0] < t * 1e-6 - self.cfg.DAQ_CLOCK_WINDOW_S:
            q.popleft()
        self.offset_now = q[0][1]
        with self._slock:
            self.samples.append((seq, t, reading, enc_a, enc_z))
        self.n_received += 1

    def clock_offset(self):
        """Host time (time.time()) of Teensy t_us = 0 for the running stream:
        host time of a sample = clock_offset() + t_us * 1e-6."""
        return self.offset_now

    def drain(self):
        """Take every sample received so far: [(t_us, reading, encA, encZ)]; the stream goes on."""
        with self._slock:
            got, self.samples = self.samples, []
        return [(t, r, a, z) for _, t, r, a, z in got]

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
        # streaming: packets arrive all the time; idle: the heartbeat keeps last_rx fresh
        return self.fatal_error is None and time.time() - self.last_rx < self.cfg.LINK_TIMEOUT_S

    def health_text(self):
        if self.fatal_error:
            return f"DAQ LINK LOST ({self.fatal_error}) - restart the program"
        if not self.link_ok():
            return "NO REPLY FROM TEENSY DAQ - check USB"
        return None

    def scale_ok(self, timeout=1.0):
        """Ask the Teensy now whether it has found the NAU7802 (it looks again on every
        '?'). True / False, or None if it didn't answer."""
        with self._cmd_lock:
            self._status_event.clear()
            try:
                self._send("?")
            except RuntimeError:
                return None
            if not self._status_event.wait(timeout):
                return None
        return "scale=1" in self.last_status

    # ---- acquisition ----
    def set_rate(self, sps):
        if sps != self.rate:
            self.command(f"R{int(sps)}", "RATE:", timeout=3.0)
            self.rate = sps

    def set_gain(self, gain):
        if gain != self.gain:
            self.command(f"G{int(gain)}", "GAIN:", timeout=3.0)
            self.gain = gain

    def calibrate(self):
        """Re-run the NAU7802's internal (AFE) calibration at the current rate + gain."""
        if self.command("C", "CAL:", timeout=3.0) != "CAL:OK":
            raise RuntimeError("NAU7802 calibration failed - check the sensor wiring")

    def start_stream(self):
        """Returns the host time that corresponds to t_us = 0 on the Teensy (the first
        estimate; clock_offset() refines it while the stream runs)."""
        with self._slock:
            self.samples = []
        self.n_received = 0
        self.crc_errors = 0
        self._t_wrap, self._t_last = 0, None
        self._clock.clear()
        self.offset_now = time.time() + self.cfg.DAQ_CMD_LATENCY_S
        self.streaming = True
        t_send = time.time()
        try:
            self.command("S", "STREAM_START", timeout=1.0)
        except RuntimeError as e:
            self.streaming = False
            if "NO_SCALE" in str(e):
                raise RuntimeError(f"NAU7802 pressure ADC not found by the Teensy - check "
                                   f"{self.cfg.NAU7802_WIRING}") from None
            raise
        self.offset_now = t_send + self.cfg.DAQ_CMD_LATENCY_S
        return self.offset_now

    def stop_stream(self):
        """Returns (the samples not drained yet [(t_us, reading, encA, encZ)], info dict)."""
        sent = None
        try:
            reply = self.command("X", "STREAM_STOP", timeout=2.0)
            sent = int(reply.split()[1])
        except (RuntimeError, ValueError, IndexError) as e:
            print(f"DAQ stop: {e}")
        finally:
            self.streaming = False
        got = self.drain()
        info = {"received": self.n_received, "sent": sent, "crc_errors": self.crc_errors,
                "dropped": (sent - self.n_received) if sent is not None else None}
        return got, info


# =============================================================================
# SECTION 7: DATA - step matching, CSV writing, quick-view plots
# =============================================================================
def enc_volts(counts):
    """AS5600 analog OUT, raw Teensy ADC counts -> volts."""
    return counts * Config.ENCODER_VREF / Config.ENCODER_ADC_MAX


def enc_value(counts, mode):
    """Raw encoder field from the Teensy -> the stored value: volts (analog) or angle
    counts 0..4095 (i2c). None = the I2C read failed (0xFFFF)."""
    if mode == "i2c":
        return None if counts == Config.ENCODER_BAD else int(counts)
    return enc_volts(counts)


def enc_full_scale(mode):
    """One shaft turn in the stored units."""
    return Config.ENCODER_I2C_COUNTS if mode == "i2c" else Config.ENCODER_VREF


def enc_unit(mode):
    return "counts" if mode == "i2c" else "V"


def split_phase(samples, host_t0, t_move_start, n_steps, step_delay_us, with_relax, mm_per_step=1.0,
                enc_mode="analog", t_move_end=None, extra_segments=()):
    """Turn a stream into rows.
    samples      : [(t_us, reading, encA, encZ)] from the Teensy (t_us = 0 at host_t0)
    t_move_start : host time the motor actually began stepping
    n_steps, step_delay_us are in MICROsteps (what the firmware runs); mm_per_step
    converts the travelled microsteps to mm (insertion depth / injection distance).
    The move ends exactly n_steps * step_delay later (fixed-cadence firmware).
    Returns (move_rows  [(time_ms, mm, bits, z_volts, a_volts, host_time)],
             relax_rows [(time_ms, bits, z_volts, a_volts, host_time)]).
    host_time (absolute) is only used for the encoder plot - it is not in the CSV.
    extra_segments: [(host start time, n_steps, step_delay_us, sign)] - further moves of the
    same axis in this recording (the RETRACTION after an insertion: sign -1), added to the
    programmed position (time 0 stays at the first move's start)."""
    dur = n_steps * step_delay_us * 1e-6
    if t_move_end is None:                 # closed loop: the real end (speed trims + make-up)
        t_move_end = t_move_start + dur
    move_rows, relax_rows = [], []
    for t_us, reading, enc_a, enc_z in samples:
        ht = host_t0 + t_us * 1e-6
        zv, av = enc_value(enc_z, enc_mode), enc_value(enc_a, enc_mode)
        if with_relax and ht > t_move_end:
            relax_rows.append(((ht - t_move_end) * 1000.0, reading, zv, av, ht))
        else:
            frac = (ht - t_move_start) / dur if dur > 0 else 1.0
            if any(ht >= ts for ts, _, _, _ in extra_segments):
                frac = 1.0                    # the follow-up move only starts once this one is done
            steps = int(min(max(frac, 0.0), 1.0) * n_steps)   # step k lands at k x delay
            for ts, n2, us2, sg in extra_segments:
                d2 = n2 * us2 * 1e-6
                f2 = (ht - ts) / d2 if d2 > 0 else 1.0
                steps += sg * int(min(max(f2, 0.0), 1.0) * n2)
            move_rows.append(((ht - t_move_start) * 1000.0, steps * mm_per_step, reading, zv, av, ht))
    return move_rows, relax_rows


def fmt_mm(v):
    """mm with up to 6 decimals (A moves 0.000625 mm per microstep), no trailing zeros."""
    out = f"{v:.6f}".rstrip("0").rstrip(".")
    return out if out not in ("", "-0") else "0"


def bits_to_kpa(bits, gain):
    """Raw signed 24-bit NAU7802 reading -> pressure (kPa), using SECTION 2:
    ADC input (V) = bits / 2^23 x (ADC_FS_FRACTION x ADC_VREF_V / gain)
    sensor output (V/psi) = SENSOR_MV_PER_V_PER_PSI x SENSOR_EXCITATION_V / 1000
    kPa = psi x KPA_PER_PSI."""
    c = Config
    volts = bits / float(1 << 23) * (c.ADC_FS_FRACTION * c.ADC_VREF_V / (gain or 1))
    psi = volts / (c.SENSOR_MV_PER_V_PER_PSI * c.SENSOR_EXCITATION_V / 1000.0)
    return c.PRESSURE_SIGN * psi * c.KPA_PER_PSI


def _mean(v):
    return sum(v) / len(v) if v else 0.0


def phase_kpa(data):
    """Pressure (kPa) for every row of every phase, aligned with the rows:
    {"watercal": [...], sid: {"ins": [...], "inj": [...], "relax": [...]}}.
    Every value is relative to the SETTLING PRESSURE (data["settle_kpa"], the end of the
    sensor warm-up) - one reference for the whole test, nothing zeroed per phase."""
    z = data.get("settle_kpa") or 0.0
    out = {}
    wc = data.get("watercal")
    if wc and wc.get("inj"):
        out["watercal"] = [bits_to_kpa(r[2], wc.get("gain", 1)) - z for r in wc["inj"]]
    for sid, s in data.get("samples", {}).items():
        g = s.get("gain", {"ins": 1, "inj": 1})
        out[sid] = {"ins": [bits_to_kpa(r[2], g["ins"]) - z for r in s["ins"]],
                    "inj": [bits_to_kpa(r[2], g["inj"]) - z for r in s["inj"]],
                    "relax": [bits_to_kpa(r[1], g["inj"]) - z for r in s["relax"]]}
    return out


def csv_columns(data):
    """Ordered (header, values) list for the wide CSV."""
    kp = phase_kpa(data)                                      # relative to the settling pressure
    mode = data.get("enc_mode", "analog")
    unit = enc_unit(mode)

    def ef(v):                                                # encoder cell
        if v is None:
            return ""                                         # failed I2C read
        return str(v) if mode == "i2c" else f"{v:.4f}"

    def move_cols(prefix, phase, dist_name, rows, kpa):      # 6 columns
        return [(f"{prefix}{phase}-TIME (ms)", [f"{r[0]:.2f}" for r in rows]),
                (f"{prefix}{phase}-{dist_name} (mm)", [fmt_mm(r[1]) for r in rows]),
                (f"{prefix}{phase}-PRESSURE (bits)", [str(r[2]) for r in rows]),
                (f"{prefix}{phase}-PRESSURE (kPa)", [f"{v:.5f}" for v in kpa]),
                (f"{prefix}{phase}-Z-ENCODER ({unit})", [ef(r[3]) for r in rows]),
                (f"{prefix}{phase}-A-ENCODER ({unit})", [ef(r[4]) for r in rows])]

    def relax_cols(prefix, rows, kpa):                       # 5 columns
        return [(f"{prefix}RELAXATION-TIME (ms)", [f"{r[0]:.2f}" for r in rows]),
                (f"{prefix}RELAXATION-PRESSURE (bits)", [str(r[1]) for r in rows]),
                (f"{prefix}RELAXATION-PRESSURE (kPa)", [f"{v:.5f}" for v in kpa]),
                (f"{prefix}RELAXATION-Z-ENCODER ({unit})", [ef(r[2]) for r in rows]),
                (f"{prefix}RELAXATION-A-ENCODER ({unit})", [ef(r[3]) for r in rows])]

    cols = []
    wc = data.get("watercal")
    if wc is not None:
        cols += move_cols("WATERCAL_", "INJECTION", "DISTANCE", wc["inj"], kp.get("watercal", []))
    for i in sorted(data.get("samples", {})):
        s = data["samples"][i]
        p = f"S{i}_"
        cols += move_cols(p, "INSERTION", "DEPTH", s["ins"], kp[i]["ins"])
        cols += move_cols(p, "INJECTION", "DISTANCE", s["inj"], kp[i]["inj"])
        cols += relax_cols(p, s["relax"], kp[i]["relax"])
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


def enc_unwrap_mm(values, mm_per_turn, full_scale=None):
    """AS5600 reading (0..full_scale = one shaft turn, then wraps) -> continuous mm.
    A jump of more than half a turn between two samples is taken as a wrap, so the
    shaft must turn less than half a turn per sample (fine up to ~max speed at 10 SPS).
    None (failed I2C read) repeats the previous position."""
    full = full_scale or Config.ENCODER_VREF
    out, acc, prev = [], 0.0, None
    for v in values:
        if v is None:
            out.append(out[-1] if out else None)
            continue
        f = v / full
        if prev is not None:
            if f - prev > 0.5:
                acc -= 1.0
            elif f - prev < -0.5:
                acc += 1.0
        prev = f
        out.append((acc + f) * mm_per_turn)
    return out


def fit_slope(t, y):
    m = len(t)
    mt, my = sum(t) / m, sum(y) / m
    den = sum((x - mt) ** 2 for x in t)
    return sum((x - mt) * (v - my) for x, v in zip(t, y)) / den if den > 0 else 0.0


def slope_series(t, y, window_s):
    """Local least-squares slope (dy/dt) over a centred window -> measured rate."""
    n = len(t)
    out = []
    j0 = j1 = 0
    for i in range(n):
        while t[j0] < t[i] - window_s / 2:
            j0 += 1
        while j1 + 1 < n and t[j1 + 1] <= t[i] + window_s / 2:
            j1 += 1
        a, b = j0, j1
        if b - a < 2:                         # too few points: widen to 2 neighbours
            a, b = max(0, i - 1), min(n - 1, i + 1)
        out.append(fit_slope(t[a:b + 1], y[a:b + 1]))
    return out


def radius_mm(volume_mm3):
    """Equivalent sphere radius (mm) of an injected volume (mm^3 = uL): V = 4/3 pi r^3."""
    return (3.0 * max(volume_mm3, 0.0) / (4.0 * 3.141592653589793)) ** (1.0 / 3.0)


def programmed_profile(x, shift):
    """Programmed position + rate at every time of trace x: out by dist over dur, then (Z
    with a retraction) back by ret['mm'] over ret['dur_s'] from ret['start_s']. shift moves
    the whole profile later (aligned to the measured start of motion)."""
    d, T, r = x["dist"], x["dur"], x.get("ret")
    pos, rate = [], []
    for tt in x["t"]:
        u = tt - shift
        p = d * min(max(u / T, 0.0), 1.0) if T > 0 else d
        v = d / T if (T > 0 and 0.0 <= u <= T) else 0.0
        if r and r.get("mm"):
            rs, rd = r["start_s"], r["dur_s"]
            p -= r["mm"] * min(max((u - rs) / rd, 0.0), 1.0) if rd > 0 else r["mm"]
            if rd > 0 and rs <= u <= rs + rd:
                v -= r["mm"] / rd
        pos.append(p)
        rate.append(v)
    return pos, rate


def encoder_profiles(data):
    """Per recorded move: measured (encoder) vs programmed travel for the axis that moves.
    Returns {"z": [traces], "a": [traces]}; trace = dict(label, sid, t, meas, plan,
    plan_rate, dist, dur, kpa, n_move, rate, dev, end_mm, mean_rate).
    Insertion = Z (stage) phases; injection (+ relaxation rows) = A."""
    c = Config
    out = {"z": [], "a": []}
    kp = phase_kpa(data)

    def build(axis, label, sid, move, relax, plan, kpa, end_s=None, ret=None):
        if not move or not plan:
            return
        dist, dur = plan[axis]
        rel0 = end_s if end_s else dur                  # relaxation starts when the move REALLY ended
        vi = 3 if axis == "z" else 4                    # volts column in move rows
        t = [r[0] / 1000.0 for r in move]
        v = [r[vi] for r in move]
        for r in relax or []:                           # relax rows: t from the move END
            t.append(rel0 + r[0] / 1000.0)
            v.append(r[vi - 1])
        mm = enc_unwrap_mm(v, c.ENC_Z_MM_PER_TURN if axis == "z" else c.ENC_A_MM_PER_TURN,
                           enc_full_scale(data.get("enc_mode", "analog")))
        first = next((m for m in mm if m is not None), None)
        if first is None:
            return                                      # encoder never answered: no trace
        mm = [first if m is None else m for m in mm]    # leading failed reads
        pre = [m for tt, m in zip(t, mm) if tt < 0]
        ref = sum(pre) / len(pre) if pre else mm[0]
        meas = [m - ref for m in mm]
        x = dict(label=label, sid=sid, t=t, meas=meas, kpa=kpa, n_move=len(move), dist=dist,
                 dur=dur, ret=ret, t_end=end_s or dur)   # t_end: when the move REALLY ended
        x["plan"], x["plan_rate"] = programmed_profile(x, 0.0)
        out[axis].append(x)

    wc = data.get("watercal")
    if wc and wc.get("plan"):
        build("a", "water-cal", 0, wc["inj"], None, wc["plan"], kp.get("watercal", []), wc.get("inj_end_s"))
    for sid in sorted(data.get("samples", {})):
        s = data["samples"][sid]
        build("z", f"S{sid}", sid, s["ins"], None, s.get("plan"), kp[sid]["ins"], s.get("ins_end_s"),
              s.get("retract"))
        build("a", f"S{sid}", sid, s["inj"], s["relax"], s.get("plan"), kp[sid]["inj"] + kp[sid]["relax"],
              s.get("inj_end_s"))

    # encoder direction: Config sign, or automatically so measured travel agrees with
    # the programmed direction (it's only a matter of which way the magnet faces)
    for axis, cfg_sign in (("z", c.ENC_Z_SIGN), ("a", c.ENC_A_SIGN)):
        tr = out[axis]
        if not tr:
            continue
        sign = cfg_sign
        if sign is None:
            # the largest excursion of each trace (NOT the last value: a retraction can bring
            # the trace back to ~0 at the end, which made the sign a coin toss)
            net = sum(max(x["meas"], key=abs) for x in tr)
            sign = -1 if net < 0 else 1
        for x in tr:
            x["meas"] = [sign * m for m in x["meas"]]
            x["rate"] = slope_series(x["t"], x["meas"], c.ENC_RATE_WINDOW_S)
            x["dev"] = [m - p for m, p in zip(x["meas"], x["plan"])]
            x["sign"] = sign
            hi = x["t_end"] + c.ENC_END_WINDOW_S
            if x["ret"]:                                  # the retraction starts right after
                hi = min(hi, x["ret"]["start_s"])
            end = [m for tt, m in zip(x["t"], x["meas"]) if x["t_end"] <= tt <= hi]
            if not end:                                   # no sample in the window: nearest one
                k = min(range(len(x["t"])), key=lambda q: abs(x["t"][q] - x["t_end"]))
                end = [x["meas"][k]]
            x["end_mm"] = sum(end) / len(end)
            f0, f1 = c.ENC_RATE_FIT_SPAN
            mid = [(tt, m) for tt, m in zip(x["t"], x["meas"]) if f0 * x["dur"] <= tt <= f1 * x["dur"]]
            if len(mid) >= 3:
                x["mean_rate"] = fit_slope([q[0] for q in mid], [q[1] for q in mid])
            else:
                x["mean_rate"] = x["end_mm"] / x["dur"] if x["dur"] > 0 else 0.0
            # measured start of motion: first crossing of ENC_START_FRACTION of the distance,
            # moved back by the time that fraction takes at the programmed rate
            frac = c.ENC_START_FRACTION
            thr = frac * x["dist"]
            x["t_start"] = 0.0
            for tt, m in zip(x["t"], x["meas"]):
                if tt > -0.1 and m >= thr:
                    x["t_start"] = tt - frac * x["dur"]
                    break
            if c.ENC_PLOT_ALIGN_START and 0.0 < x["t_start"] < c.ENC_ALIGN_MAX_LAG_S:
                # programmed trajectory timed from the REAL start of motion: the deviation is
                # then tracking error only (the start delay is reported as start_lag_ms)
                x["plan"], x["plan_rate"] = programmed_profile(x, x["t_start"])
                x["dev"] = [m - p for m, p in zip(x["meas"], x["plan"])]
            x["start_lag_ms"] = round(x["t_start"] * 1000, 1)
    return out


def _first_last_legend(ax, lines, extra=()):
    """Legend with only the FIRST sample (blue) and the LAST sample (red) [+ extras]."""
    if not lines:
        return
    keep = [lines[0]] + ([lines[-1]] if len(lines) > 1 else []) + list(extra)
    ax.legend(keep, [h.get_label() for h in keep], loc=PlotStyle.LEGEND_LOC,
              fontsize=PlotStyle.LEGEND_FONT_SIZE, frameon=PlotStyle.LEGEND_FRAME)


def _sample_colors(ids):
    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("first_to_last",
                                             [PlotStyle.FIRST_SAMPLE_COLOR, PlotStyle.LAST_SAMPLE_COLOR])
    n = len(ids)
    return {sid: cmap(k / (n - 1) if n > 1 else 0.0) for k, sid in enumerate(ids)}


def encoder_plot(plt, data, png_dir, stem, prof=None):
    """3 x 3 figure: rows = travel (mm), rate (mm/s), deviation (measured - programmed);
    columns = Z stage during INSERTION | A plunger during INJECTION | A during the hold.
    Measured = AS5600 encoder (solid, first sample blue -> last red, water-cal grey);
    programmed = the commanded constant-speed move (black dashed)."""
    prof = prof or encoder_profiles(data)
    if not prof["z"] and not prof["a"]:
        return None
    S = PlotStyle
    grid = S.GRID_COLOR
    sids = sorted(data.get("samples", {}))
    colors = _sample_colors(sids)

    def col(x):
        return S.WATERCAL_COLOR if x["sid"] == 0 else colors[x["sid"]]

    fig, axs = plt.subplots(3, 3, figsize=S.ENCODER_FIGSIZE, dpi=S.DPI, sharex="col", constrained_layout=True)
    titles = dict(enumerate(S.ENCODER_COLUMN_TITLES))
    for ci, axis in enumerate(("z", "a", "a")):
        tr = [x for x in prof[axis] if x["sid"]]                # samples only - no water-cal
        if ci == 2:
            tr = [x for x in tr if x["t"][-1] > x["dur"] + 0.5]
        a_mm, a_rate, a_dev = axs[0][ci], axs[1][ci], axs[2][ci]
        a_mm.set_title(titles[ci], fontsize=S.PANEL_TITLE_FONT_SIZE)
        if not tr:
            a_mm.text(0.5, 0.5, "no data", transform=a_mm.transAxes, ha="center", color=S.WATERCAL_COLOR)
            continue
        sample_lines = []
        for x in tr:
            ln, = a_mm.plot(x["t"], x["meas"], color=col(x), lw=S.ENC_TRAVEL_LW, label=x["label"])
            if x["sid"]:
                sample_lines.append(ln)
            a_rate.plot(x["t"], x["rate"], color=col(x), lw=S.ENC_RATE_LW)
            a_dev.plot(x["t"], x["dev"], color=col(x), lw=S.ENC_DEVIATION_LW)
        ref = max(tr, key=lambda q: len(q["t"]))        # programmed profile (same for all)
        prog, = a_mm.plot(ref["t"], ref["plan"], S.PROGRAMMED_STYLE, lw=S.PROGRAMMED_LW,
                          label=S.PROGRAMMED_LABEL)
        a_rate.plot(ref["t"], ref["plan_rate"], S.PROGRAMMED_STYLE, lw=S.PROGRAMMED_LW)
        a_dev.axhline(0, color="k", ls="--", lw=S.ZERO_LINE_LW)
        for ax in (a_mm, a_rate, a_dev):
            ax.axvline(ref["dur"], color=grid, lw=1, zorder=0)
            ax.grid(True, color=grid, lw=S.GRID_LW)
        a_mm.set_ylabel(S.ENCODER_ROW_LABELS[0])
        a_rate.set_ylabel(S.ENCODER_ROW_LABELS[1])
        a_dev.set_ylabel(S.ENCODER_ROW_LABELS[2])
        a_dev.set_xlabel(S.ENCODER_XLABEL)
        if ci == 1:                                  # zoom on the move itself
            pad = max(0.2, 0.25 * ref["dur"])
            a_dev.set_xlim(-pad, ref["dur"] + pad)
        if ci == 2:                                  # the hold only: drift / creep after the move
            a_dev.set_xlim(ref["dur"], max(x["t"][-1] for x in tr))
            for axx, key in ((a_mm, "meas"), (a_dev, "dev")):
                ys = [v for q in tr for tt, v in zip(q["t"], q[key]) if tt >= ref["dur"] + 0.05]
                if ys:
                    lo, hi = min(ys), max(ys)
                    m = max(1e-4, 0.15 * (hi - lo))
                    axx.set_ylim(lo - m, hi + m)
            ys = [v for q in tr for tt, v in zip(q["t"], q["rate"]) if tt >= ref["dur"] + 0.1]
            if ys:
                m = max(abs(min(ys)), abs(max(ys)), 1e-3) * 1.3
                a_rate.set_ylim(-m, m)
        if ci < 2:                                   # (hold column: same colours, no legend)
            _first_last_legend(a_mm, sample_lines, [prog])
    fig.suptitle(S.ENCODER_TITLE, fontsize=S.SUPTITLE_FONT_SIZE)
    p = png_dir / f"{stem}_{S.FILE_ENCODERS}.png"
    fig.savefig(p)
    plt.close(fig)
    for axis, name in (("z", "Z insertion"), ("a", "A injection")):     # numbers -> terminal
        for x in prof[axis]:
            print(f"   encoder {name} {x['label']}: end {x['end_mm']:.4f} / {x['dist']:.4f} mm, "
                  f"rate {x['mean_rate']:.4g} / {x['dist'] / x['dur'] if x['dur'] else 0:.4g} mm/s")
    return p


def pressure_figure(plt, data, prof, png_dir, stem, measured, collapsed=False):
    """1 x 3 pressure figure, commanded (measured=False) or encoder-measured (True):
      pressure vs insertion depth | pressure vs time (injection + relaxation) |
      pressure vs equivalent-sphere radius of the injected volume (injection).
    Pressure = kPa relative to the settling pressure. collapsed=True (commanded depth /
    volume): each insertion curve minus its first value and each injection curve minus its
    first value (COLLAPSE_ZERO_SAMPLES), so every sample starts at 0 kPa."""
    samples = data.get("samples", {})
    ids = sorted(samples)
    if not ids:
        return None
    colors = _sample_colors(ids)
    kp = phase_kpa(data)
    area = Config.nl_per_mm(data.get("syringe", Config.DEFAULT_SYRINGE)) / 1000.0   # mm^3 per mm
    ztr = {x["sid"]: x for x in prof["z"] if x["sid"]}
    atr = {x["sid"]: x for x in prof["a"] if x["sid"]}
    S = PlotStyle
    lw = S.PRESSURE_LW
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=S.PRESSURE_FIGSIZE, dpi=S.DPI, constrained_layout=True)
    l1, l2, l3 = [], [], []
    for sid in ids:
        s, col, lab = samples[sid], colors[sid], f"S{sid}"
        dur = s.get("inj_end_s") or (s.get("plan") or {}).get("a", (None, None))[1]
        if dur is None:
            dur = s["inj"][-1][0] / 1000.0 if s["inj"] else 0.0
        if not measured:
            depth = [r[1] for r in s["ins"]]
            t_inj = [r[0] / 1000.0 for r in s["inj"]] + [dur + r[0] / 1000.0 for r in s["relax"]]
            vol = [r[1] * area for r in s["inj"]]
            k_ins, k_all, k_inj = kp[sid]["ins"], kp[sid]["inj"] + kp[sid]["relax"], kp[sid]["inj"]
            if collapsed:
                n0 = max(1, Config.COLLAPSE_ZERO_SAMPLES)
                z_ins = _mean(k_ins[:n0]) if k_ins else 0.0     # first value before inserting
                z_inj = _mean(k_inj[:n0]) if k_inj else 0.0     # first value before injecting
                k_ins = [v - z_ins for v in k_ins]
                k_all = [v - z_inj for v in k_all]
                k_inj = [v - z_inj for v in k_inj]
        else:
            if sid not in ztr or sid not in atr:
                continue
            z, a = ztr[sid], atr[sid]
            depth, k_ins = z["meas"], z["kpa"]
            t_inj = [tt - a["t_start"] for tt in a["t"]]
            vol = [m * area for m in a["meas"][:a["n_move"]]]
            k_all, k_inj = a["kpa"], a["kpa"][:a["n_move"]]
        h, = ax1.plot(depth, k_ins, color=col, lw=lw, label=lab); l1.append(h)
        h, = ax2.plot(t_inj, k_all, color=col, lw=lw, label=lab); l2.append(h)
        h, = ax3.plot([radius_mm(v) for v in vol], k_inj, color=col, lw=lw, label=lab); l3.append(h)
    if not l1:
        plt.close(fig)
        return None
    # grey line where the retraction starts = the programmed insertion depth (on the measured
    # figure too, so you can see where the turn-around SHOULD have been)
    with_ret = [sid for sid in ids if (samples[sid].get("retract") or {}).get("mm")]
    if with_ret:
        s0 = samples[with_ret[0]]
        depth = (s0.get("plan") or {}).get("z", (None,))[0]
        if depth:
            ax1.axvline(depth, color=S.RETRACTION_LINE_COLOR, lw=S.RETRACTION_LINE_LW, zorder=0)
            ax1.text(depth, 0.98, S.RETRACTION_LABEL, transform=ax1.get_xaxis_transform(),
                     color=S.RETRACTION_TEXT_COLOR, fontsize=S.RETRACTION_FONT_SIZE,
                     ha="right", va="top", rotation=90)
    ax1.set_xlabel(S.PRESSURE_XLABELS[0])
    ax2.set_xlabel(S.PRESSURE_XLABELS[1])
    ax3.set_xlabel(S.PRESSURE_XLABELS[2])
    for ax, lines in ((ax1, l1), (ax2, l2), (ax3, l3)):
        ax.set_ylabel(S.COLLAPSED_YLABEL if collapsed else S.PRESSURE_YLABEL)
        ax.grid(True, color=S.GRID_COLOR, lw=S.GRID_LW)
        _first_last_legend(ax, lines)
    title, ending = ((S.TITLE_COLLAPSED, S.FILE_COLLAPSED) if collapsed else
                     (S.TITLE_MEASURED, S.FILE_MEASURED) if measured else
                     (S.TITLE_RECOVERED, S.FILE_RECOVERED))
    fig.suptitle(title, fontsize=S.SUPTITLE_FONT_SIZE)
    p = png_dir / f"{stem}_{ending}.png"
    fig.savefig(p)
    plt.close(fig)
    return p


def make_plots(data, png_dir, stem):
    """Quick-view PNGs. Returns list of saved paths (empty if matplotlib missing)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("⚠ matplotlib not installed - no plots. Install: sudo apt install python3-matplotlib")
        return []

    S = PlotStyle
    plt.rcParams.update(S.RC_PARAMS)
    saved = []
    try:
        prof = encoder_profiles(data)
    except Exception as e:
        print(f"✗ Encoder conversion failed: {e}")
        prof = {"z": [], "a": []}

    # recovered (programmed depth / volume), collapsed (same, every curve starts at 0 kPa),
    # encoder-measured
    for title, measured, collapsed in ((S.TITLE_RECOVERED, False, False),
                                       (S.TITLE_COLLAPSED, False, True),
                                       (S.TITLE_MEASURED, True, False)):
        try:
            p = pressure_figure(plt, data, prof, png_dir, stem, measured, collapsed)
            if p:
                saved.append(p)
        except Exception as e:
            print(f"✗ {title} plot failed: {e}")

    wc = data.get("watercal")
    if wc and wc["inj"]:
        kp = phase_kpa(data).get("watercal", [])
        area = Config.nl_per_mm(data.get("syringe", Config.DEFAULT_SYRINGE)) / 1000.0
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=S.WATERCAL_FIGSIZE, dpi=S.DPI, constrained_layout=True)
        ax1.plot([r[0] / 1000.0 for r in wc["inj"]], kp, color=S.WATERCAL_LINE_COLOR, lw=S.WATERCAL_LW)
        ax1.set_xlabel(S.WATERCAL_XLABELS[0])
        ax2.plot([radius_mm(r[1] * area) for r in wc["inj"]], kp, color=S.WATERCAL_LINE_COLOR, lw=S.WATERCAL_LW)
        ax2.set_xlabel(S.WATERCAL_XLABELS[1])
        for ax in (ax1, ax2):
            ax.set_ylabel(S.WATERCAL_YLABEL)
            ax.grid(True, color=S.GRID_COLOR, lw=S.GRID_LW)
        fig.suptitle(S.WATERCAL_TITLE, fontsize=S.WATERCAL_TITLE_FONT_SIZE)
        p = png_dir / f"{stem}_{S.FILE_WATERCAL}.png"
        fig.savefig(p)
        plt.close(fig)
        saved.append(p)

    # ---- AS5600 encoders -> measured travel + rate vs the programmed profile ----
    try:
        p = encoder_plot(plt, data, png_dir, stem, prof)
        if p:
            saved.append(p)
    except Exception as e:                       # never lose the other plots over this one
        print(f"✗ Encoder plot failed: {e}")
    return saved


# =============================================================================
# SECTION 8: TEST ROUTINE + RETURN-TO-HOME + JOGGING
# =============================================================================
class ClosedLoop:
    """Encoder feedback for ONE recorded single-axis move (Z insertion or A injection).
    tick() runs from MotorLink.move_abs every ~CL_PERIOD_S and does two jobs:

    1. LOST STEPS (distance): the firmware reports how many steps it has made ("VP done
       total"); the encoder says how far the shaft really went at that same moment. The
       difference = steps that were lost. They are APPENDED to the running move right
       away ("V us extra"), so the move keeps going and ends at the programmed distance.
    2. LAG (timing): PID on e = programmed - measured position, with the programmed
       trajectory starting when the motion REALLY starts (detected on the encoder, so the
       USB/start delay isn't counted as error). Outside +/- tol the speed is trimmed
       (0.5x..2x, but never past the axis maximum - then it is 'saturated').
    finish() checks once more when stopped and makes up any remaining shortfall."""

    def __init__(self, run, axis, motor, dist_mm, us0, params, host_t0, direction=1):
        c = run.cfg
        self.run, self.cfg, self.link, self.daq = run, c, run.link, run.daq
        self.dirn = direction                   # +1 insert / infuse, -1 backwards (retraction)
        self.axis, self.motor, self.dist, self.us0 = axis, motor, dist_mm, us0
        self.mpm = c.mm_per_micro(axis)
        self.v0 = self.mpm * 1e6 / us0                   # programmed speed (mm/s)
        self.dur = dist_mm / self.v0 if dist_mm else 0.0
        self.p = dict(params)
        self.host_t0 = host_t0
        self.t_cmd = None                       # host time the move was commanded
        self.t_on = None                        # host time the motion really started
        self.full = enc_full_scale(self.daq.enc_mode)
        self.lead = c.ENC_Z_MM_PER_TURN if axis == "z" else c.ENC_A_MM_PER_TURN
        enc_res = self.lead / (c.ENCODER_I2C_COUNTS if self.daq.enc_mode == "i2c" else c.ENCODER_ADC_MAX)
        self.thr = max(c.CL_MOVING_MICROSTEPS * self.mpm, c.CL_MOVING_ENC_COUNTS * enc_res)   # "moving" (mm)
        self.k = 0                              # next DAQ sample to read
        self.acc, self.prev_f, self.raw = 0.0, None, None
        self.hist = []                          # (host time, raw mm) - recent encoder samples
        self.ref = None
        self.integral, self.prev_en, self.prev_t = 0.0, None, None
        self.last_tick = 0.0
        self.us_sent = us0
        self.added = 0                          # steps appended DURING the move
        self.extra_steps = 0                    # make-up steps AFTER the move
        self.ticks = self.saturated = 0
        self.max_extra = int(math.ceil(c.CL_MAX_CORRECTION * dist_mm / self.mpm)) if dist_mm else 0
        self.gave_up = None                     # reason, if the encoder can't be trusted
        self.fac_min = self.fac_max = 1.0
        self.trace = []                         # (t, measured, speed factor, added, loss, delay ms)
        self.dev_abs = []                       # |e| samples while moving (mm)
        self.final_error_mm = None
        self.prog_seen = None                   # last progress report used
        self.delta = None                       # calibrated step-count -> encoder delay (s)
        self.v_fw = None                        # step rate the motor board REALLY runs (mm/s)
        self.prev_prog = None
        self.loss = 0.0                         # fraction of steps being lost (feed-forward)

    # -- encoder --
    def _ingest(self):
        smp = self.daq.samples
        n = len(smp)
        while self.k < n:
            seq, t_us, _, ea, ez = smp[self.k]
            self.k += 1
            v = enc_value(ez if self.axis == "z" else ea, self.daq.enc_mode)
            if v is None:
                continue                         # failed I2C read
            f = v / self.full
            if self.prev_f is not None:
                if f - self.prev_f > 0.5:
                    self.acc -= 1.0
                elif f - self.prev_f < -0.5:
                    self.acc += 1.0
            self.prev_f = f
            self.raw = (self.acc + f) * self.lead
            self.hist.append((self.host_t0 + t_us * 1e-6, self.raw))
        if len(self.hist) > 4000:
            del self.hist[:2000]

    def _signed(self, raw):
        """Raw encoder mm -> travel in the programmed direction (None until the sign is known)."""
        if raw is None or self.ref is None:
            return None
        d = raw - self.ref
        sign = self.run.enc_sign.get(self.axis)
        if sign is None:                         # learn which way the magnet counts
            c = self.cfg
            if abs(d) >= max(self.thr, min(c.CL_SIGN_LEARN_MM, c.CL_SIGN_LEARN_FRACTION * self.dist)):
                sign = self.run.enc_sign[self.axis] = (1 if d > 0 else -1) * self.dirn
            else:
                return None
        return self.dirn * sign * d              # travel in THIS move's direction

    def measured(self):
        return self._signed(self.raw)

    def _measured_at(self, t):
        """Encoder travel interpolated at host time t (None if outside the history)."""
        h = self.hist
        if len(h) < 2 or t < h[0][0] or t > h[-1][0]:
            return None
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        (t0, r0), (t1, r1) = h[lo], h[hi]
        r = r0 + (r1 - r0) * ((t - t0) / (t1 - t0) if t1 > t0 else 0.0)
        return self._signed(r)

    def start(self, t_cmd):
        """Called just before G is sent: zero = the current (pre-roll) encoder reading."""
        self._ingest()
        self.ref = self.raw
        self.t_cmd = t_cmd
        self.link.move_progress = None

    # -- during the move --
    def tick(self):
        c = self.cfg
        now = time.time()
        if self.gave_up or now - self.last_tick < c.CL_PERIOD_S:
            return
        self.last_tick = now
        self._ingest()
        x = self.measured()
        if x is None or not self.hist:
            return
        t_s = self.hist[-1][0]                   # time of the newest encoder sample

        # motion start (the programmed trajectory is timed from here)
        if self.t_on is None:
            if x >= self.thr:
                for tt, rr in self.hist:
                    m = self._signed(rr)
                    if m is not None and m >= self.thr and tt >= self.t_cmd - 0.05:
                        self.t_on = tt - self.thr / self.v0
                        break
            elif now - self.t_cmd > max(c.CL_START_TIMEOUT_S, 5 * self.thr / self.v0):
                self.t_on = self.t_cmd           # never saw it start: use the command time

        extra = 0
        # 1) lost steps: firmware step count vs encoder at the same moment
        prog = self.link.move_progress
        if prog is not None and prog is not self.prog_seen:
            self.prog_seen = prog
            done, total, t_r = prog
            x_at = self._measured_at(t_r - c.CL_SYNC_LAG_S)
            # the step rate the board is REALLY making (it can fall behind the request)
            if self.prev_prog is not None and t_r > self.prev_prog[2] and done > self.prev_prog[0]:
                v = (done - self.prev_prog[0]) * self.mpm / (t_r - self.prev_prog[2])
                k = c.CL_RATE_SMOOTHING
                self.v_fw = v if self.v_fw is None else (1 - k) * self.v_fw + k * v
            self.prev_prog = prog
            v_now = self.v_fw or self.mpm * 1e6 / self.us_sent
            if x_at is not None and self.t_on is not None and done > 0:
                gap = done * self.mpm - x_at - self.added * self.mpm
                if self.delta is None:
                    # timing gap between the two boards, from the START of motion only (not
                    # affected by lost steps): when the board began stepping (its first
                    # report, run back by the steps it had made) vs when the encoder moved
                    t_fw0 = (t_r - c.CL_SYNC_LAG_S) - done * self.us0 * 1e-6
                    self.delta = max(0.0, min(c.CL_MAX_SYNC_S, self.t_on - t_fw0))
                else:
                    lost = gap - self.delta * v_now          # steps really lost, not yet made up
                    if (self.added * self.mpm + lost) > c.CL_MAX_CORRECTION * self.dist + self.p["tol"]:
                        self._give_up(f"encoder is {self.added * self.mpm + lost:.3f} mm behind the "
                                      f"step count (> {c.CL_MAX_CORRECTION:.0%} of the move)")
                        return
                    margin = self.p["tol"] + v_now * c.CL_SYNC_UNCERTAINTY_S
                    if lost > margin:                        # conservative: never overshoot
                        extra = int(math.floor((lost - margin) / self.mpm))
                        extra = min(extra, self.max_extra - self.added)
                        if extra > 0:
                            self.added += extra
                        else:
                            extra = 0
                    if done > 0:                             # share of steps being lost
                        self.loss = max(0.0, min(c.CL_MAX_LOST_FRACTION, (self.added * self.mpm + max(0.0, lost))
                                                 / (done * self.mpm)))

        # 2) lag: PID speed trim against the programmed trajectory
        us = self.us_sent
        if self.t_on is not None:
            t = t_s - self.t_on
            x_prog = self.dist * min(max(t / self.dur, 0.0), 1.0) if self.dur > 0 else self.dist
            e = x_prog - x                       # + = behind the programmed trajectory
            if 0 < t < self.dur:
                self.dev_abs.append(abs(e))
            tol, tau = self.p["tol"], c.CL_TAU_S
            # error outside the dead band, in units of "tau seconds of programmed travel"
            en = (e - math.copysign(tol, e)) / (self.v0 * tau) if abs(e) > tol else 0.0
            dt = (t - self.prev_t) if self.prev_t is not None else 0.0
            if dt > 0:
                self.integral = max(-c.CL_INTEGRAL_LIMIT,
                                    min(c.CL_INTEGRAL_LIMIT, self.integral + en * dt / tau))
            if self.prev_en is not None and en * self.prev_en < 0:
                self.integral = 0.0              # crossed the trajectory: no wind-up overshoot
            deriv = tau * (en - self.prev_en) / dt if (dt > 0 and self.prev_en is not None) else 0.0
            self.prev_en, self.prev_t = en, t
            s = self.p["kp"] * en + self.p["ki"] * self.integral + self.p["kd"] * deriv
            # feed-forward: if a share of the steps is being lost, step that much faster
            fac = (1.0 + s) / (1.0 - self.loss)
            fac = max(c.CL_MIN_SPEED_FACTOR, min(c.CL_MAX_SPEED_FACTOR, fac))
            want = self.us0 / fac
            us = max(c.MIN_STEP_US, int(round(want)))
            self.ticks += 1
            self.fac_min = min(self.fac_min, self.us0 / us)
            self.fac_max = max(self.fac_max, self.us0 / us)
            if want < c.MIN_STEP_US - 0.5:
                self.saturated += 1              # wanted to go faster than the axis can

        if self.t_on is not None:
            self.trace.append((round(t_s - self.t_on, 4), round(x, 4), round(self.us0 / us, 3), self.added,
                               round(self.loss, 3), None if self.delta is None else round(self.delta * 1000, 2)))
        self.link.send(f"V {us} {extra}")        # always: keeps the progress reports coming
        self.us_sent = us

    def _give_up(self, why):
        """Stop correcting this move: back to the programmed speed, nothing more added."""
        self.gave_up = why
        self.link.send(f"V {self.us0} 0")
        self.us_sent = self.us0
        print(f"⚠ closed loop OFF for this move: {why} - check the encoder (magnet, wiring, lead)")

    # -- after the programmed steps --
    def finish(self, abort_event):
        """Make up any shortfall > tol that is left (stationary check, exact). Returns host
        time of the last DONE (or None if nothing was added)."""
        c = self.cfg
        t_done = None
        for _ in range(0 if self.gave_up else c.CL_FINAL_TRIES):
            run_sleep(c.CL_SETTLE_S, abort_event)
            self._ingest()
            x = self.measured()
            if x is None:
                break
            short = self.dist - x
            self.final_error_mm = -short
            if short <= self.p["tol"]:
                break
            n = int(round(short / self.mpm))
            budget = self.max_extra - self.added - self.extra_steps
            if n > budget:
                self._give_up(f"still {short:.3f} mm short after the move (> {c.CL_MAX_CORRECTION:.0%} "
                              "of it) - not adding more")
                break
            if n <= 0:
                break
            cur = list(self.link.pos)
            cur[self.motor - 1] += self.run.move_dir[self.axis] * self.dirn * n
            self.link.set_delay(self.motor, self.us0)
            _, t_done = self.link.move_abs(cur, self.us0, abort_event)
            self.extra_steps += n
        self._ingest()
        x = self.measured()
        if x is not None:
            self.final_error_mm = x - self.dist
        return t_done

    def summary(self):
        c = self.cfg
        return {"axis": self.axis, "params": self.p,
                "added_steps_during_move": self.added,
                "extra_steps": self.added + self.extra_steps,
                "extra_mm": round((self.added + self.extra_steps) * self.mpm, 5),
                "makeup_steps_after_move": self.extra_steps,
                "final_error_mm": None if self.final_error_mm is None else round(self.final_error_mm, 5),
                "start_lag_ms": None if self.t_on is None else round((self.t_on - self.t_cmd) * 1000, 1),
                "mean_abs_deviation_mm": round(sum(self.dev_abs) / len(self.dev_abs), 5) if self.dev_abs else None,
                "max_abs_deviation_mm": round(max(self.dev_abs), 5) if self.dev_abs else None,
                "speed_factor_min": round(self.fac_min, 4), "speed_factor_max": round(self.fac_max, 4),
                "speed_saturated_pct": round(100.0 * self.saturated / self.ticks, 1) if self.ticks else 0.0,
                "at_axis_max_speed": self.us0 <= c.MIN_STEP_US * 1.02,
                "lost_fraction": round(self.loss, 4),
                "gave_up": self.gave_up,
                "timing_gap_ms": None if self.delta is None else round(self.delta * 1000, 2),
                "updates": self.ticks,
                # thinned tick log for diagnosis: (t s, measured mm, speed factor, steps added,
                # lost fraction, timing gap ms)
                "trace": self.trace[::max(1, len(self.trace) // 150)]}


def run_sleep(t, abort_event):
    end = time.time() + t
    while time.time() < end:
        if abort_event.is_set():
            raise Aborted("Stopped by user")
        time.sleep(0.01)


class PalpationRun(threading.Thread):
    """Executes the full test in a background thread so the GUI (and its STOP
    button) stays responsive. All test parameters are in mm and mm/s.

    Z is the STAGE (sample / cup) moving under a fixed needle: inserting = stage UP
    (Config.Z_INSERT_SIGN), clearing = stage DOWN. Before ANY XY travel the stage
    drops to the safe height = past the CUP EXIT (cup_lift_m below the station Z)
    and past every saved sample height.

    Sequence (the needle never WITHDRAWS liquid at a sample):
      FILLING STATION: travel there, water-cal injection (no insertion),
                       purge PURGE_MM, withdraw (refill) back to the starting level
      each sample:     travel above it, insert (stage up, insertion rate), inject (A,
                       injection rate), hold RELAX_HOLD_S, pull the needle out
                       (stage down), travel to the FILLING STATION (needle goes into
                       the cup), purge PURGE_MM, withdraw (refill) to the starting level
    All moves between positions run at the FINE jog speed (TRAVEL_JOG_TIER).
    """

    def __init__(self, link, cfg, home, samples, depth_mm=None, insert_mm_s=None,
                 inject_mm=None, inject_mm_s=None, daq=None, rate=None, gain=None,
                 xy_homed=False, cup_lift_m=0, syringe=None, cl=None,
                 retract_mm=None, retract_mm_s=None):
        super().__init__(daemon=True)
        self.xy_homed = xy_homed
        self.link, self.cfg, self.daq = link, cfg, daq
        self.home, self.samples = list(home), [list(s) for s in samples]   # [b1, b2, z, a] (home = filling station)
        self.depth_mm = cfg.DEFAULT_INSERT_DEPTH_MM if depth_mm is None else float(depth_mm)
        self.insert_mm_s = cfg.DEFAULT_INSERT_RATE_MM_S if insert_mm_s is None else float(insert_mm_s)
        self.inject_mm = cfg.DEFAULT_INJECT_DIST_MM if inject_mm is None else float(inject_mm)
        self.inject_mm_s = cfg.DEFAULT_INJECT_RATE_MM_S if inject_mm_s is None else float(inject_mm_s)
        self.rate = rate or cfg.DEFAULT_RATE            # ONE sample rate + gain for the whole test
        self.gain = gain or cfg.DEFAULT_GAIN            # (one continuous stream, see SECTION 1)
        self.rate_ins = self.rate_inj = self.rate       # (per-phase names kept for the data files)
        self.gain_ins = self.gain_inj = self.gain
        self.retract_mm = cfg.DEFAULT_RETRACT_DIST_MM if retract_mm is None else float(retract_mm)
        self.retract_mm_s = cfg.DEFAULT_RETRACT_RATE_MM_S if retract_mm_s is None else float(retract_mm_s)
        self.abort_event = threading.Event()
        self.status = "Starting..."
        self.phase_idx = 0                 # 0 = water-cal, 1..N = samples
        self.n_phases = len(self.samples) + (1 if cfg.WATER_CAL_ENABLED else 0)
        self.done_phases = 0
        self.finished = False
        self.error = None
        self.data = {"watercal": None, "samples": {}, "syringe": syringe or cfg.DEFAULT_SYRINGE,
                     "enc_mode": daq.enc_mode if daq else "analog"}
        self.csv_path = None
        self.png_paths = []
        self.daq_notes = []
        self.syringe = syringe or cfg.DEFAULT_SYRINGE
        # closed loop (needs the DAQ): {"z": {...}, "a": {...}} like Config.CL_DEFAULTS
        self.cl = {k: dict(v) for k, v in (cl or cfg.CL_DEFAULTS).items()}
        if not daq:
            for v in self.cl.values():
                v["on"] = False
        self.enc_sign = {"z": cfg.ENC_Z_SIGN, "a": cfg.ENC_A_SIGN}    # learned on the first move
        self.move_dir = {"z": cfg.Z_INSERT_SIGN, "a": cfg.INJ_DISPENSE_SIGN}
        self.cl_log = []                   # one summary per controlled move
        self.xy_offset_total = [0, 0]      # belt shift applied by re-homing (GUI applies it too)
        self.xy_drift = []                 # per re-home: (label, d_belt1, d_belt2) microsteps
        self.events = []                   # (host time, text) - the test timeline
        self.t_start = self.t_end = None   # host times of the start / end of the test
        self.xy_homed_at_start = xy_homed
        self.trial_dir = None
        self.params_txt = None
        self.points = {}                   # phase label -> pressure points recorded
        self.acts = []                     # (host time, what the rig is doing) - labels the
                                           # continuous recording ("S2 injection", "S2 travel", ...)
        self.settle_bits = self.settle_kpa = None   # SETTLING PRESSURE (end of the warm-up)
        self.cont_path = self.cont_file = self.cont_writer = None   # <trial>_continuous.csv
        self.cont_rows = 0
        self.rail_rows = 0                 # samples pinned at the ADC limit (clipped)
        self.warming = False               # True during the sensor warm-up (RUN screen)
        self.cup_lift_m = abs(int(cup_lift_m or 0))   # stage drop (microsteps) to clear the cup
        self.plan = []                     # estimated seconds per phase (see plan_phases)
        self.eta_end = None                # host time the test should finish (countdown)
        self.phase_started = []            # host time each phase began (estimate vs actual)

        # what the firmware actually runs: microsteps + microstep delays
        self.pen_m = cfg.mm_to_micro("z", self.depth_mm)
        self.inj_m = cfg.mm_to_micro("a", self.inject_mm)
        self.purge_m = cfg.mm_to_micro("a", cfg.PURGE_MM)
        self.ins_us = cfg.us_for_mm_s("z", self.insert_mm_s)
        self.ret_m = min(self.pen_m, cfg.mm_to_micro("z", self.retract_mm))   # retraction (microsteps)
        self.ret_us = cfg.us_for_mm_s("z", self.retract_mm_s)
        self.inj_us = cfg.us_for_mm_s("a", self.inject_mm_s)
        self.travel_us = cfg.travel_delays()   # FINE jog speeds for moving between positions
        self._cur_us = {}                  # motor -> delay currently set in the firmware

        # safe travel height: the stage at least cup_lift_m below the station (needle out
        # of the cup) and at/below every saved sample height, + SAFE_Z_EXTRA_LIFT_MM
        cup_exit = self.home[2] - cfg.Z_INSERT_SIGN * self.cup_lift_m
        safe = cup_exit
        for z in [self.home[2]] + [s[2] for s in self.samples]:
            safe = z_more_clear(safe, z)
        self.safe_z = safe - cfg.Z_INSERT_SIGN * cfg.mm_to_micro("z", cfg.SAFE_Z_EXTRA_LIFT_MM)

    def abort(self):
        self.abort_event.set()
        self.link.stop_all()

    # ---- primitive moves (keep the other axes where they are) ----
    def _set_us(self, motor, us):
        if self._cur_us.get(motor) != us:
            self.link.set_delay(motor, us)
            self._cur_us[motor] = us

    def _move(self, b1=None, b2=None, z=None, a=None, us=None):
        cur = list(self.link.pos)
        tgt = [cur[0] if b1 is None else b1, cur[1] if b2 is None else b2,
               cur[2] if z is None else z, cur[3] if a is None else a]
        return self.link.move_abs(tgt, us, self.abort_event)

    def _z(self, z, us=None):
        """Z move at `us` per microstep (default: Z travel speed)."""
        us = us or self.travel_us[3]
        self._set_us(3, us)
        return self._move(z=z, us=us)

    def _a(self, a, us=None):
        """A move at `us` per microstep (default: purge / refill speed)."""
        us = us or self.travel_us[4]
        self._set_us(4, us)
        return self._move(a=a, us=us)

    def _travel_to(self, p, label):
        """Stage down to the safe height, move XY (straight line), stage to p's height.
        Ends EXACTLY at p's X, Y, Z (absolute step counts)."""
        self._act(f"{label}: travel")
        cur_z = self.link.pos[2]
        travel_z = z_more_clear(self.safe_z, cur_z)     # stage down to the safe height first
        if cur_z != travel_z:
            self.status = f"{label}: lowering the stage to the safe height"
            self._z(travel_z)
        self.status = f"{label}: XY travel"
        self._set_us(1, self.travel_us[1])
        self._set_us(2, self.travel_us[2])
        self._move(b1=p[0], b2=p[1], us=self.travel_us[1])
        self.status = f"{label}: raising the stage to the saved Z"
        self._z(p[2])

    # ---- time estimate: walks the same sequence as run() without moving anything ----
    def plan_phases(self, start_pos):
        """Estimated seconds for [start + water-cal, sample 1, ..., sample N] from
        start_pos (raw counts). The last entry includes saving + plotting."""
        c = self.cfg
        us = dict(self.travel_us)
        pos = list(start_pos)
        ramp_start, ramp_len = c.xy_us(c.XY_RAMP_START_US), c.xy_steps(c.XY_RAMP_FULLSTEPS or 0)

        def move(tgt, delay):
            d = [abs(t - p) for t, p in zip(tgt, pos)]
            lead = max(range(4), key=lambda k: d[k])
            n = d[lead]
            pos[:] = tgt
            if n == 0:
                return c.EST_CMD_OVERHEAD_S
            t = n * delay * 1e-6
            if lead < 2 and c.XY_RAMP_START_US and ramp_len and ramp_start > delay:   # X/Y ramps
                L = min(ramp_len * (ramp_start - delay) / ramp_start, n / 2.0)
                t += L * (ramp_start - delay) * 1e-6
            return t + c.EST_CMD_OVERHEAD_S

        def z(v, delay=None):
            return move([pos[0], pos[1], v, pos[3]], delay or us[3])

        def a(v, delay=None):
            return move([pos[0], pos[1], pos[2], v], delay or us[4])

        def travel(p):
            t = 0.0
            tz = z_more_clear(self.safe_z, pos[2])
            if tz != pos[2]:
                t += z(tz)
            t += move([p[0], p[1], pos[2], pos[3]], us[1])
            return t + z(p[2])

        def recorded(t_move, hold, rate=None):
            if not self.daq:
                return t_move + hold
            return c.PRE_ROLL_S + t_move + (hold if hold > 0 else c.POST_ROLL_S)   # stream never stops

        def purge_refill(a_start):
            return a(pos[3] + c.INJ_DISPENSE_SIGN * self.purge_m) + a(a_start)

        def rehome():
            t = 0.0
            tz = z_more_clear(self.safe_z, pos[2])
            if tz != pos[2]:
                t += z(tz)
            x, y = belts_to_xy(pos[0], pos[1])                  # full steps from the corner
            bo = c.HOMING_FINAL_BACKOFF_STEPS
            t += (abs(x) + abs(y) + 2 * bo) * c.HOMING_DELAY_US * 1e-6 + c.EST_REHOME_OVERHEAD_S
            pos[0], pos[1] = xy_to_belts(bo, bo)
            return t

        phases = []
        t = c.EST_START_S + (c.EST_DAQ_START_S + c.DAQ_WARMUP_S if self.daq else 0.0)   # + sensor warm-up
        if c.REHOME_XY_EACH_SAMPLE and not self.xy_homed:
            t += rehome()
        t += travel(self.home)
        a0 = pos[3]
        if c.WATER_CAL_ENABLED:
            t += recorded(a(a0 + c.INJ_DISPENSE_SIGN * self.inj_m, self.inj_us), 0.0, self.rate_inj)
            t += purge_refill(a0)
        phases.append(t)
        for p in self.samples:
            a0 = pos[3]
            t = travel(p)
            tm = z(p[2] + c.Z_INSERT_SIGN * self.pen_m, self.ins_us)
            if self.ret_m > 0:
                tm += z(p[2] + c.Z_INSERT_SIGN * (self.pen_m - self.ret_m), self.ret_us)
            t += recorded(tm, 0.0, self.rate_ins)
            t += recorded(a(a0 + c.INJ_DISPENSE_SIGN * self.inj_m, self.inj_us), c.RELAX_HOLD_S, self.rate_inj)
            t += z(p[2])
            if c.REHOME_XY_EACH_SAMPLE:
                t += rehome()
            t += travel(self.home)
            t += purge_refill(a0)
            phases.append(t)
        if self.daq:
            phases[-1] += c.EST_SAVE_PLOTS_S
        return phases

    def _rehome_xy(self, label):
        """Stage down to the safe height, find both X-Y limit switches, re-zero there and
        shift every saved position by the same amount (they stay on the same physical
        spots). The shift = how far X-Y had drifted (0 if no steps were lost)."""
        c = self.cfg
        self._act(f"X-Y re-home ({label})")
        cur_z = self.link.pos[2]
        tz = z_more_clear(self.safe_z, cur_z)
        if tz != cur_z:
            self.status = f"Re-homing X-Y ({label}): lowering the stage"
            self._z(tz)
        self.status = f"Re-homing X-Y ({label})"
        self._event(f"X-Y re-home ({label}) starts")
        hx = HomeXY(self.link, c, lift_z_to=None)
        hx.abort_event = self.abort_event            # STOP RUN stops the homing too
        hx.run()                                     # synchronously, in this thread
        self._cur_us = {}                            # homing changed the motor delays
        for m, us in self.travel_us.items():
            self._set_us(m, us)
        if hx.error or hx.offset is None:
            raise Aborted(f"X-Y re-home failed ({label}): {hx.error or 'no zero'}")
        d1, d2 = hx.offset[0], hx.offset[1]
        for p in [self.home] + self.samples:
            p[0] -= d1
            p[1] -= d2
        self.xy_offset_total[0] += d1
        self.xy_offset_total[1] += d2
        if self.xy_homed:                            # drift only means something once homed
            dx, dy = belts_to_xy(d1, d2)
            self.xy_drift.append({"when": label, "belt_microsteps": [d1, d2],
                                  "x_mm": round(dx * c.mm_per_full("xy"), 4),
                                  "y_mm": round(dy * c.mm_per_full("xy"), 4)})
            print(f"   X-Y re-home {label}: drift X {dx * c.mm_per_full('xy'):+.3f} mm, "
                  f"Y {dy * c.mm_per_full('xy'):+.3f} mm")
        self._event(f"X-Y re-home ({label}) done" + (
            f" - drift X {self.xy_drift[-1]['x_mm']:+.3f} mm, Y {self.xy_drift[-1]['y_mm']:+.3f} mm"
            if self.xy_drift and self.xy_drift[-1]["when"] == label else ""))
        self.xy_homed = True

    def _event(self, text):
        self.events.append((time.time(), text))

    def remaining_s(self):
        """Countdown value for the GUI (None before the run has planned itself)."""
        if self.eta_end is None:
            return None
        return max(0.0, self.eta_end - time.time())

    def _anchor(self, phase):
        """Re-aim the countdown at the start of a phase (absorbs any drift so far)."""
        self.phase_started.append(time.time())
        if self.plan:
            self.eta_end = time.time() + sum(self.plan[phase:])

    def _sleep(self, t):
        end = time.time() + t
        while time.time() < end:
            if self.abort_event.is_set():
                raise Aborted("Stopped by user")
            time.sleep(0.02)

    def _purge_and_refill(self, a_start, tag):
        """Only ever called with the needle IN the filling station: a quick purge
        (A infuse PURGE_MM), then withdraw back to a_start - i.e. refill the
        injected volume + the purge."""
        c = self.cfg
        self.status = f"{tag}: purging {c.PURGE_MM:g} mm at the filling station"
        self._act(f"{tag}: purge")
        self._a(self.link.pos[3] + c.INJ_DISPENSE_SIGN * self.purge_m)
        back = abs(self.link.pos[3] - a_start) * c.mm_per_micro("a")
        self.status = f"{tag}: refilling (withdraw {back:.3f} mm)"
        self._act(f"{tag}: refill")
        self._a(a_start)

    def _cl_move(self, axis, target, n_steps, step_us, label, host_t0, direction=1):
        """One single-axis move to `target` (absolute count), closed-loop if enabled.
        direction: +1 = the axis' normal direction (insert / infuse), -1 = backwards
        (the retraction). Returns (host time sent, host time it really ended or None)."""
        c = self.cfg
        motor = 3 if axis == "z" else 4
        self._set_us(motor, step_us)
        tgt = list(self.link.pos)
        tgt[motor - 1] = target
        p = self.cl.get(axis, {})
        if not (p.get("on") and self.daq):
            t_send, t_done = self.link.move_abs(tgt, step_us, self.abort_event)
            return t_send, None
        ctrl = ClosedLoop(self, axis, motor, n_steps * c.mm_per_micro(axis), step_us, p, host_t0,
                          direction=direction)
        self.status = f"{label} (closed loop)"
        ctrl.start(time.time() + c.MOTOR_CMD_LATENCY_S)
        t_send, t_done = self.link.move_abs(tgt, step_us, self.abort_event, on_tick=ctrl.tick,
                                            slow_factor=1.0 / c.CL_MIN_SPEED_FACTOR)
        t_extra = ctrl.finish(self.abort_event)
        summ = ctrl.summary()
        summ["phase"] = label
        self.cl_log.append(summ)
        print(f"   closed loop {label}: start lag {summ['start_lag_ms']} ms, speed "
              f"x{summ['speed_factor_min']}..x{summ['speed_factor_max']}"
              f"{' (SATURATED at axis max ' + str(summ['speed_saturated_pct']) + ' %)' if summ['speed_saturated_pct'] else ''}, "
              f"+{summ['added_steps_during_move']} steps during / +{summ['makeup_steps_after_move']} after, "
              f"mean |dev| {summ['mean_abs_deviation_mm']} mm, final error {summ['final_error_mm']} mm")
        return t_send, (t_extra or t_done)

    def _recorded(self, move_fn, axis, n_steps, step_us, hold_s, gain, label, target=None,
                  rate=None, then=None):
        """Record one phase around one motor move (+ optional follow-up move + hold), cut out
        of the continuous pressure stream (v01: the sensor never stops during a test; `gain`
        and `rate` are the test-wide ones and are only kept in the call for readability).
        n_steps / step_us: microsteps and microstep delay of that move; target: its absolute
        target count. then = (n_steps, step_us, label): a second move of the same axis
        BACKWARDS by n_steps from wherever it is now, in the same recording (the retraction
        after an insertion). Relative on purpose: closed-loop make-up steps on the insertion
        mean the step counter is ahead of the stage - an absolute target would over-retract.
        Returns (move_rows, relax_rows); with no DAQ just does the motion.
        self.last_move_end_s = when the (first) move really ended (s after its start);
        self.last_then = {"mm", "dur_s", "start_s"} of the follow-up move (or None)."""
        c = self.cfg
        self.last_move_end_s = n_steps * step_us * 1e-6
        self.last_then = None
        if not self.daq:
            self._event(f"{label}: move starts")
            move_fn()
            self._event(f"{label}: move ends")
            if then:
                n2, us2, lab2 = then
                motor = 3 if axis == "z" else 4
                tg2 = self.link.pos[motor - 1] - self.move_dir[axis] * n2
                self._event(f"{lab2}: move starts")
                self._cl_move(axis, tg2, n2, us2, lab2, 0.0, direction=-1)
                self._event(f"{lab2}: move ends")
            if hold_s > 0:
                self.status = f"{label}: holding {hold_s:.1f}s"
                self._sleep(hold_s)
                self._event(f"{label}: relaxation hold ends ({hold_s:g} s)")
            return [], []
        if not self.daq.link_ok():
            raise Aborted(self.daq.health_text() or "DAQ link not OK")
        # the sensor has been streaming since the warm-up: everything before this phase goes
        # to the continuous CSV, and the phase is the window [t_phase0, t_phase1] of the stream
        self._flush_stream()
        t_phase0 = time.time()
        self._act(label)
        t_end = None
        segs = []
        self._sleep(c.PRE_ROLL_S)
        self._event(f"{label}: move starts ({n_steps * c.mm_per_micro(axis):g} mm, "
                    f"{c.mm_s_for_us(axis, step_us):.4g} mm/s)")
        if target is not None:
            t_send, t_end = self._cl_move(axis, target, n_steps, step_us, label,
                                          self.daq.clock_offset())
        else:
            t_send, _ = move_fn()
        self._event(f"{label}: move ends")
        if then:
            n2, us2, lab2 = then
            motor = 3 if axis == "z" else 4
            tg2 = self.link.pos[motor - 1] - self.move_dir[axis] * n2   # relative: see above
            self.status = f"{lab2}: {n2 * c.mm_per_micro(axis):g} mm at {c.mm_s_for_us(axis, us2):.4g} mm/s"
            self._event(f"{lab2}: move starts ({n2 * c.mm_per_micro(axis):g} mm, "
                        f"{c.mm_s_for_us(axis, us2):.4g} mm/s)")
            self._act(lab2)
            t2, _ = self._cl_move(axis, tg2, n2, us2, lab2, self.daq.clock_offset(), direction=-1)
            segs.append((t2 + c.MOTOR_CMD_LATENCY_S, n2, us2, -1))
            self._event(f"{lab2}: move ends")
        if hold_s > 0:
            self.status = f"{label}: holding {hold_s:.1f}s (relaxation)"
            self._act(f"{label}: relaxation")
            self._sleep(hold_s)
        else:
            self._sleep(c.POST_ROLL_S)
        t_phase1 = time.time()
        samples, host_t0 = self._phase_samples(t_phase0, t_phase1, label)
        t0m = t_send + c.MOTOR_CMD_LATENCY_S
        if t_end is not None:
            self.last_move_end_s = t_end - t0m
        if segs:
            ts, n2, us2, _ = segs[0]
            self.last_then = {"mm": n2 * c.mm_per_micro(axis), "dur_s": n2 * us2 * 1e-6,
                              "start_s": ts - t0m}
        self.points[label] = len(samples)
        if hold_s > 0:
            self._event(f"{label}: relaxation hold ends ({hold_s:g} s)")
        self._event(f"{label}: recording stops - {len(samples)} pressure points")
        return split_phase(samples, host_t0, t0m,
                           n_steps, step_us, with_relax=hold_s > 0,
                           mm_per_step=c.mm_per_micro(axis), enc_mode=self.daq.enc_mode,
                           t_move_end=t_end, extra_segments=segs)

    # ---- continuous pressure stream (v01) ----
    def _act(self, label):
        """Note what the rig starts doing now - labels the continuous recording."""
        self.acts.append((time.time(), label))

    def _daq_begin(self):
        """Set the sensor up ONCE for the whole test: rate, gain, ADC calibration, then start
        the one continuous stream and let it settle for DAQ_WARMUP_S with nothing moving.
        The mean of the warm-up's last SETTLE_WINDOW_S = the SETTLING PRESSURE: every kPa
        value of the test is relative to it."""
        c = self.cfg
        self.status = f"Pressure sensor: {self.rate} SPS, gain x{self.gain}, calibrating"
        self.daq.set_rate(self.rate)
        self.daq.set_gain(self.gain)
        self.daq.calibrate()
        self._open_continuous()
        self.daq.start_stream()
        self._act("sensor warm-up")
        self.warming = True
        self._event(f"Pressure sensor on: {self.rate} SPS, gain x{self.gain}; warm-up {c.DAQ_WARMUP_S:g} s")
        end = time.time() + c.DAQ_WARMUP_S
        while True:
            left = end - time.time()
            if left <= 0:
                break
            self.status = f"Pressure sensor warm-up: {left:.0f} s (keep the rig still)"
            self._sleep(min(0.25, left))
        t1, off = time.time(), self.daq.clock_offset()
        bits = [r for _, t, r, _, _ in list(self.daq.samples) if off + t * 1e-6 >= t1 - c.SETTLE_WINDOW_S]
        if not bits:
            raise Aborted("No pressure data from the Teensy during the sensor warm-up")
        self.settle_bits = sum(bits) / len(bits)
        rail = c.ADC_RAIL_FRACTION * (1 << 23)
        if abs(self.settle_bits) > rail:
            raise Aborted(f"Pressure ADC is PINNED at its {'lower' if self.settle_bits < 0 else 'upper'} limit "
                          f"({self.settle_bits:.0f} bits) - a sensor wire is open or loose (E+/E-/A+/A- at "
                          f"the NAU7802), or the gain x{self.gain} is too high. Check with "
                          f"DigitalPalpation_daqMonitor.py before testing.")
        self.settle_kpa = bits_to_kpa(self.settle_bits, self.gain)
        self.data["settle_kpa"] = self.settle_kpa
        self.data["settle_bits"] = self.settle_bits
        self._act("sensor ready")
        self.warming = False
        self._event(f"Pressure sensor warm-up done: settling pressure = {self.settle_bits:.0f} bits "
                    f"(mean of the last {c.SETTLE_WINDOW_S:g} s)")

    def _phase_samples(self, t0, t1, label):
        """The samples of the running stream between host times t0 and t1 (waits briefly for
        the last ones to arrive). Returns ([(t_us, reading, encA, encZ)], host time of t_us 0)."""
        deadline = time.time() + 1.0
        while True:
            off, smp = self.daq.clock_offset(), list(self.daq.samples)
            if (smp and off + smp[-1][1] * 1e-6 >= t1) or time.time() > deadline:
                break
            time.sleep(0.01)
        rows = [(t, r, a, z) for _, t, r, a, z in smp if t0 <= off + t * 1e-6 <= t1]
        if not rows:
            raise Aborted(f"No pressure data from the Teensy during {label} - check its USB cable")
        return rows, off

    def _open_continuous(self):
        if not (self.cfg.SAVE_CONTINUOUS_CSV and self.csv_dir):
            return
        unit = enc_unit(self.daq.enc_mode)
        self.cont_path = self.csv_dir / f"{self.stem}_continuous.csv"
        self.cont_file = open(self.cont_path, "w", newline="")
        self.cont_writer = csv.writer(self.cont_file)
        self.cont_writer.writerow(["TIME (s)", "ACTIVITY", "PRESSURE (bits)", "PRESSURE (kPa)",
                                   f"Z-ENCODER ({unit})", f"A-ENCODER ({unit})"])

    def _flush_stream(self, rows=None):
        """Move every sample received so far out of memory into the continuous CSV, each
        labelled with what the rig was doing at that moment."""
        if rows is None:
            rows = self.daq.drain()
        if not (self.cont_writer and rows):
            return
        mode, off = self.daq.enc_mode, self.daq.clock_offset()
        times = [t for t, _ in self.acts]
        for t, r, a, z in rows:
            k = bisect.bisect_right(times, off + t * 1e-6) - 1
            ev, av = enc_value(z, mode), enc_value(a, mode)
            self.cont_writer.writerow([
                f"{t * 1e-6:.5f}", self.acts[k][1] if k >= 0 else "",
                r, "" if self.settle_kpa is None else f"{bits_to_kpa(r, self.gain) - self.settle_kpa:.5f}",
                "" if ev is None else (ev if mode == "i2c" else f"{ev:.4f}"),
                "" if av is None else (av if mode == "i2c" else f"{av:.4f}")])
        self.cont_rows += len(rows)
        rail = self.cfg.ADC_RAIL_FRACTION * (1 << 23)
        self.rail_rows += sum(1 for _, r, _, _ in rows if abs(r) > rail)
        self.cont_file.flush()

    def _daq_end(self):
        """Stop the one stream (end of test or STOP): last samples -> continuous CSV."""
        if not (self.daq and self.daq.streaming):
            return
        self._flush_stream()
        rest, info = self.daq.stop_stream()
        self._flush_stream(rest)
        if info["dropped"] or info["crc_errors"]:
            note = f"pressure stream: {info['dropped']} dropped, {info['crc_errors']} CRC errors"
            self.daq_notes.append(note)
            print("⚠ " + note)
        self._event(f"Pressure sensor off: {info['received']} samples in the continuous recording")
        if self.rail_rows:
            note = (f"{self.rail_rows} of {self.cont_rows} pressure samples were PINNED at the ADC limit "
                    f"(clipped) - lower the gain or check the sensor wiring")
            self.daq_notes.append(note)
            print("⚠ " + note)

    def _close_continuous(self):
        if self.cont_file:
            try:
                self.cont_file.close()
            except OSError:
                pass
            self.cont_file = self.cont_writer = None

    def _phase_plan(self):
        """Programmed profile of each recorded move, for the encoder plot:
        axis -> (distance mm, move duration s)."""
        c = self.cfg
        return {"z": (self.pen_m * c.mm_per_micro("z"), self.pen_m * self.ins_us * 1e-6),
                "a": (self.inj_m * c.mm_per_micro("a"), self.inj_m * self.inj_us * 1e-6)}

    def _write_params_txt(self):
        """TEST_PARAMS_TRIAL_<n>.txt - the exact parameters of this run (everything on the
        FINAL CHECK screen and more) + a timeline in hh:mm:ss since the start and UTC."""
        c = self.cfg
        name = f"TEST_PARAMS_{self.trial_dir.name}.txt"
        path = (self.trial_dir if c.PARAMS_TXT_IN_TRIAL_FOLDER else self.trial_dir.parent) / name
        t0 = self.t_start or (self.events[0][0] if self.events else time.time())
        t1 = self.t_end or time.time()

        def utc(t):
            return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + " UTC"

        def local(t):
            return datetime.fromtimestamp(t).strftime("%Y-%m-%d %H:%M:%S")

        def mm_pos(p):
            x, y = belts_to_xy(p[0], p[1])
            return (f"X {x * c.mm_per_full('xy'):8.2f}  Y {y * c.mm_per_full('xy'):8.2f}  "
                    f"Z {p[2] * c.mm_per_micro('z'):8.3f}  A {p[3] * c.mm_per_micro('a'):8.3f} mm")

        def spd(axis, us):
            return f"{c.mm_s_for_us(axis, us):9.4f} mm/s   ({us} us per microstep)"

        nlmm = c.nl_per_mm(self.syringe)
        ins_mm_s, inj_mm_s = c.mm_s_for_us("z", self.ins_us), c.mm_s_for_us("a", self.inj_us)
        L = []
        add = L.append
        bar = "=" * 78

        def section(title):
            add("")
            add(bar)
            add(title)
            add(bar)

        add(f"DIGITAL PALPATION - TEST PARAMETERS - {self.trial_dir.name}")
        add(f"Result      : {'COMPLETE' if self.error is None else 'STOPPED - ' + str(self.error)}")
        add(f"Data folder : {self.trial_dir}")

        section("TIME")
        add(f"Start (UTC)        : {utc(t0)}")
        add(f"End   (UTC)        : {utc(t1)}")
        add(f"Start (local)      : {local(t0)}")
        add(f"End   (local)      : {local(t1)}")
        add(f"Duration           : {fmt_hms(t1 - t0)}  (hh:mm:ss)")
        add(f"Estimated duration : {fmt_hms(sum(self.plan)) if self.plan else '-'}")

        section("SAMPLES")
        add(f"Samples            : {len(self.samples)}" + (" + water-cal" if c.WATER_CAL_ENABLED else ""))
        add(f"Samples completed  : {self.done_phases - (1 if c.WATER_CAL_ENABLED and self.done_phases else 0)}"
            f" of {len(self.samples)}")
        add(f"Filling station    : {mm_pos(self.home)}")
        for i, p in enumerate(self.samples, 1):
            add(f"{'S' + str(i):<19s}: {mm_pos(p)}")
        add("  (positions in the frame at the END of the test; X-Y relative to the limit-switch corner"
            + (")" if self.xy_homed else " only if homed)"))

        section("TEST SETTINGS (as on the FINAL CHECK screen)")
        add(f"Syringe            : {self.syringe}  (inner diameter {c.syringe_id(self.syringe):g} mm, "
            f"{nlmm:.1f} nL per mm of plunger travel)")
        add(f"Insertion depth    : {self.depth_mm:g} mm  (= {self.pen_m} Z microsteps)")
        add(f"Insertion rate     : {self.insert_mm_s:g} mm/s requested, {ins_mm_s:.4f} mm/s actual "
            f"({self.ins_us} us per microstep)")
        if self.ret_m > 0:
            add(f"Retraction         : {self.ret_m * c.mm_per_micro('z'):g} mm right after the insertion "
                f"(= {self.ret_m} Z microsteps); injection happens at "
                f"{(self.pen_m - self.ret_m) * c.mm_per_micro('z'):g} mm depth")
            add(f"Retraction rate    : {self.retract_mm_s:g} mm/s requested, "
                f"{c.mm_s_for_us('z', self.ret_us):.4f} mm/s actual ({self.ret_us} us per microstep)")
        else:
            add("Retraction         : none")
        add(f"Injection distance : {self.inject_mm:g} mm  (= {self.inj_m} A microsteps)")
        add(f"Injection rate     : {self.inject_mm_s:g} mm/s requested, {inj_mm_s:.4f} mm/s actual "
            f"({self.inj_us} us per microstep)")
        add(f"Injected volume    : {fmt_volume(nlmm * self.inject_mm)}  per injection")
        add(f"Injection flow     : {fmt_flow(nlmm * inj_mm_s)}")
        add(f"Relaxation hold    : {c.RELAX_HOLD_S:g} s")
        add(f"Purge              : {c.PURGE_MM:g} mm  ({fmt_volume(nlmm * c.PURGE_MM)}), then refill")
        add(f"Cup lift           : {self.cup_lift_m * c.mm_per_micro('z'):.3f} mm")
        add(f"Water-cal          : {'on' if c.WATER_CAL_ENABLED else 'off'}")
        add(f"X-Y homed at start : {'yes' if self.xy_homed_at_start else 'no (homed by the test)'}")
        add(f"X-Y re-home        : {'after every sample' if c.REHOME_XY_EACH_SAMPLE else 'off'}")

        section("MOTOR SPEEDS")
        tr = self.travel_us
        add(f"X-Y travel (FINE)  : {spd('xy', tr[1])}")
        add(f"Z travel (FINE)    : {spd('z', tr[3])}")
        add(f"Z insertion        : {spd('z', self.ins_us)}")
        if self.ret_m > 0:
            add(f"Z retraction       : {spd('z', self.ret_us)}")
        add(f"A injection        : {spd('a', self.inj_us)}")
        add(f"A purge / refill   : {spd('a', tr[4])}")
        add(f"X-Y homing         : {spd('xy', c.xy_us(c.HOMING_DELAY_US))}")
        add(f"X-Y speed ramp     : from {c.XY_RAMP_START_US} us/full step over {c.XY_RAMP_FULLSTEPS} full steps")
        add(f"Microstepping      : X/Y 1/{c.XY_MICROSTEPS}, Z 1/{c.Z_MICROSTEPS}, A 1/{c.A_MICROSTEPS}")
        add(f"Mechanics          : X/Y {c.mm_per_full('xy'):g} mm, Z {c.mm_per_full('z'):g} mm, "
            f"A {c.mm_per_full('a'):g} mm per full step")

        section("CLOSED-LOOP CONTROL (PID)")
        for axis in ("z", "a"):
            p = self.cl.get(axis, {})
            if p.get("on"):
                add(f"{axis.upper()}                  : ON   tolerance {p['tol']:g} mm   "
                    f"Kp {p['kp']:g}   Ki {p['ki']:g}   Kd {p['kd']:g}")
            else:
                add(f"{axis.upper()}                  : off (open loop)")
        add(f"Speed limits       : {c.CL_MIN_SPEED_FACTOR:g}x - {c.CL_MAX_SPEED_FACTOR:g}x programmed, "
            f"update every {c.CL_PERIOD_S * 1000:g} ms, up to {c.CL_FINAL_TRIES} make-up moves")
        for d in self.cl_log:
            add(f"  {d['phase']}:")
            add(f"      start lag {d['start_lag_ms']} ms | speed x{d['speed_factor_min']}..x{d['speed_factor_max']}"
                + (f" (wanted faster than the axis max {d['speed_saturated_pct']} % of the time)"
                   if d['speed_saturated_pct'] else ""))
            add(f"      lost steps made up: {d['added_steps_during_move']} while moving + "
                f"{d['makeup_steps_after_move']} after = {d['extra_mm']} mm")
            add(f"      deviation while moving: mean {d['mean_abs_deviation_mm']} mm, max {d['max_abs_deviation_mm']} mm"
                f" | final error {d['final_error_mm']} mm")
            if d.get("gave_up"):
                add(f"      !! CLOSED LOOP GAVE UP: {d['gave_up']} - move finished as programmed")

        section("DATA ACQUISITION")
        if self.daq:
            add(f"Sample rate        : {self.rate} SPS - ONE continuous recording for the whole test")
            add(f"Gain               : x{self.gain}")
            add(f"Sensor warm-up     : {c.DAQ_WARMUP_S:g} s at the start (after the ADC calibration, nothing "
                "moving)")
            add(f"Settling pressure  : " + (f"{self.settle_bits:.0f} bits = mean of the last {c.SETTLE_WINDOW_S:g} s "
                                          "of the warm-up = 0 kPa for every kPa value of the test"
                                          if self.settle_bits is not None else "-"))
            add(f"Pre-roll / post    : {c.PRE_ROLL_S:g} s before each move, {c.POST_ROLL_S:g} s after "
                "insertion / water-cal")
            add(f"Encoders           : AS5600, {self.daq.enc_mode} mode")
            add(f"Pressure (kPa)     : bits / 2^23 x ({c.ADC_FS_FRACTION:g} x {c.ADC_VREF_V:g} V / gain) / "
                f"({c.SENSOR_MV_PER_V_PER_PSI:g} mV/V/psi x {c.SENSOR_EXCITATION_V:g} V) x {c.KPA_PER_PSI:g}"
                + (" x -1 (PRESSURE_SIGN)" if c.PRESSURE_SIGN < 0 else "")
                + "; relative to the settling pressure (no other zeroing)")
            add(f"Pressure points    : {sum(self.points.values())} in total")
            for k, v in self.points.items():
                add(f"  {k:<30s}: {v}")
            for note in self.daq_notes:
                add(f"  WARNING: {note}")
        else:
            add("DAQ not connected - nothing recorded")

        if self.xy_drift:
            section("X-Y DRIFT FOUND AT EACH RE-HOME (removed by the re-home)")
            for d in self.xy_drift:
                add(f"  {d['when']:<12s}: X {d['x_mm']:+.3f} mm, Y {d['y_mm']:+.3f} mm  "
                    f"(belts {d['belt_microsteps'][0]:+d}, {d['belt_microsteps'][1]:+d} microsteps)")

        section("TIMELINE  (hh:mm:ss since the start  |  UTC)")
        for t, text in self.events:
            add(f"  {fmt_hms(t - t0)}  |  {utc(t)}  |  {text}")

        section("FILES")
        if self.csv_path:
            add(f"  {self.csv_path}")
            if self.cont_path:
                add(f"  {self.cont_path}   ({self.cont_rows} samples, the whole test)")
            add(f"  {self.csv_dir / (self.stem + '_settings.json')}")
        for p in self.png_paths:
            add(f"  {p}")
        add("")
        path.write_text("\n".join(L))
        self.params_txt = path
        print(f"✓ Test parameters saved: {path}")

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
        if not getattr(self, "started", None):
            self.started = datetime.now().isoformat(timespec="seconds")
        info = {
            "csv": self.csv_path.name, "started": self.started,
            "filling_station": self.home, "samples": self.samples,
            "insertion_depth_mm": self.depth_mm,
            "insertion_rate_mm_s": {"requested": self.insert_mm_s,
                                    "actual": round(c.mm_s_for_us("z", self.ins_us), 5)},
            "injection_distance_mm": self.inject_mm,
            "injection_rate_mm_s": {"requested": self.inject_mm_s,
                                    "actual": round(c.mm_s_for_us("a", self.inj_us), 5)},
            "syringe": {"name": self.syringe, "inner_diameter_mm": c.syringe_id(self.syringe),
                        "nL_per_mm": round(c.nl_per_mm(self.syringe), 3),
                        "injection_volume_nL": round(c.nl_per_mm(self.syringe) * self.inject_mm, 3),
                        "injection_flow_nL_s": round(c.nl_per_mm(self.syringe) * c.mm_s_for_us("a", self.inj_us), 3)},
            "cup_lift_mm": round(self.cup_lift_m * c.mm_per_micro("z"), 4),
            "estimated_duration_s": round(sum(self.plan), 1),
            "z_insert_sign": c.Z_INSERT_SIGN,
            "closed_loop": self.cl,
            "rehome_xy_each_sample": c.REHOME_XY_EACH_SAMPLE,
            "results": {"closed_loop_moves": self.cl_log, "xy_drift_at_rehome": self.xy_drift},
            "trial": self.trial_dir.name,
            "pressure_conversion": {"adc_vref_v": c.ADC_VREF_V, "adc_fs_fraction": c.ADC_FS_FRACTION,
                                    "sensor_mV_per_V_per_psi": c.SENSOR_MV_PER_V_PER_PSI,
                                    "excitation_v": c.SENSOR_EXCITATION_V, "kPa_per_psi": c.KPA_PER_PSI,
                                    "reference": "settling pressure (end of the warm-up)",
                                    "sign": c.PRESSURE_SIGN},
            "purge_mm": c.PURGE_MM, "relax_hold_s": c.RELAX_HOLD_S,
            "sample_rate_sps": self.rate,
            "gain": self.gain,
            "continuous_stream": {"warmup_s": c.DAQ_WARMUP_S, "settle_window_s": c.SETTLE_WINDOW_S,
                                  "settling_pressure_bits": self.settle_bits,
                                  "continuous_csv": self.cont_path.name if self.cont_path else None},
            "retraction_mm": self.ret_m * c.mm_per_micro("z"),
            "retraction_rate_mm_s": {"requested": self.retract_mm_s,
                                     "actual": round(c.mm_s_for_us("z", self.ret_us), 5)},
            "pre_roll_s": c.PRE_ROLL_S, "post_roll_s": c.POST_ROLL_S, "water_cal": c.WATER_CAL_ENABLED,
            "travel_mm_s": {"xy": round(c.mm_s_for_us("xy", self.travel_us[1]), 3),
                            "z": round(c.mm_s_for_us("z", self.travel_us[3]), 3),
                            "a_purge_refill": round(c.mm_s_for_us("a", self.travel_us[4]), 4)},
            "mechanics": {"xy_mm_per_full_step": c.mm_per_full("xy"), "z_mm_per_full_step": c.mm_per_full("z"),
                          "a_mm_per_full_step": c.mm_per_full("a"),
                          "microsteps": {"xy": c.XY_MICROSTEPS, "z": c.Z_MICROSTEPS, "a": c.A_MICROSTEPS}},
            "xy_homed": self.xy_homed,
            "encoders": ({"mode": "i2c", "A": "AS5600 RAW ANGLE on Wire2 (SDA 25, SCL 24)",
                          "Z": "AS5600 RAW ANGLE on Wire1 (SDA 17, SCL 16)",
                          "counts_per_turn": c.ENCODER_I2C_COUNTS, "failed_read": "blank cell"}
                         if self.daq.enc_mode == "i2c" else
                         {"mode": "analog", "A": "AS5600 analog OUT on Teensy pin 15",
                          "Z": "AS5600 analog OUT on Teensy pin 16",
                          "volts_per_count": c.ENCODER_VREF / c.ENCODER_ADC_MAX}),
            "note": ("positions are raw firmware counts [M1 belt, M2 belt, Z, A] (microsteps). X/Y are "
                     "relative to the limit-switch corner if xy_homed is true, otherwise to where that "
                     "session started"),
        }
        (self.csv_dir / f"{self.stem}_settings.json").write_text(json.dumps(info, indent=2))

    def run(self):
        c = self.cfg
        try:
            self.link.set_enabled(True)        # energized for the whole test (holds during relaxation)
            for m, us in self.travel_us.items():
                self._set_us(m, us)
            time.sleep(0.05)
            self.t_start = time.time()
            self._event("Test started")
            self.link.query_pos()
            self.plan = self.plan_phases(self.link.pos)
            self._anchor(0)
            if self.daq:
                stamp = datetime.now()
                self.trial_dir, self.csv_dir, self.png_dir = c.trial_dirs(stamp)
                self.stem = f"{c.FILE_PREFIX}_{stamp:%m_%d_%Y}_{self.trial_dir.name}"
                self.csv_path = self.csv_dir / f"{self.stem}.csv"
                self._save_settings()
                self._daq_begin()                  # sensor on ONCE + warm-up; streams until the end

            # ---- 0. X-Y must be relative to the limit switches for the re-homing ----
            if c.REHOME_XY_EACH_SAMPLE and not self.xy_homed:
                self._rehome_xy("start of test")

            # ---- 1. always start from the FILLING STATION ----
            self._travel_to(self.home, "Going to the filling station")
            self._event("At the filling station")

            # ---- 2. water calibration in the filling station (injection only) ----
            if c.WATER_CAL_ENABLED:
                tag = "WATER-CAL @ filling station"
                a_start = self.link.pos[3]
                self.status = f"{tag}: injecting {self.inject_mm:g} mm at {self.inject_mm_s:g} mm/s"
                inj_rows, _ = self._recorded(
                    lambda: self._a(a_start + c.INJ_DISPENSE_SIGN * self.inj_m, self.inj_us),
                    "a", self.inj_m, self.inj_us, 0.0, self.gain_inj, tag,
                    target=a_start + c.INJ_DISPENSE_SIGN * self.inj_m, rate=self.rate_inj)
                if self.daq:
                    self.data["watercal"] = {"inj": inj_rows, "plan": self._phase_plan(),
                                             "gain": self.gain_inj, "inj_end_s": self.last_move_end_s}
                    self._save()
                self._purge_and_refill(a_start, tag)
                self._event("Water-cal: purge + refill done")
                self.done_phases += 1

            # ---- 3. samples (the injector only ever INFUSES here) ----
            n = len(self.samples)
            for i, p in enumerate(self.samples, start=1):
                self.phase_idx = i
                tag = f"Sample {i}/{n}"
                a_start = self.link.pos[3]
                self._anchor(i)
                self._event(f"S{i}: travel to the sample starts")

                self._travel_to(p, tag)
                self._event(f"S{i}: above the sample")

                self.status = f"{tag}: inserting {self.depth_mm:g} mm at {self.insert_mm_s:g} mm/s"
                then = None
                if self.ret_m > 0:                 # back the needle out a little, still recording
                    then = (self.ret_m, self.ret_us, f"{tag} retraction")
                ins_rows, _ = self._recorded(
                    lambda: self._z(p[2] + c.Z_INSERT_SIGN * self.pen_m, self.ins_us),
                    "z", self.pen_m, self.ins_us, 0.0, self.gain_ins, f"{tag} insertion",
                    target=p[2] + c.Z_INSERT_SIGN * self.pen_m, rate=self.rate_ins, then=then)
                ins_end = self.last_move_end_s
                retract = self.last_then

                self.status = f"{tag}: injecting {self.inject_mm:g} mm at {self.inject_mm_s:g} mm/s"
                inj_rows, relax_rows = self._recorded(
                    lambda: self._a(a_start + c.INJ_DISPENSE_SIGN * self.inj_m, self.inj_us),
                    "a", self.inj_m, self.inj_us, c.RELAX_HOLD_S, self.gain_inj, f"{tag} injection",
                    target=a_start + c.INJ_DISPENSE_SIGN * self.inj_m, rate=self.rate_inj)
                if self.daq:
                    self.data["samples"][i] = {"ins": ins_rows, "inj": inj_rows, "relax": relax_rows,
                                               "plan": self._phase_plan(),
                                               "gain": {"ins": self.gain_ins, "inj": self.gain_inj},
                                               "ins_end_s": ins_end, "inj_end_s": self.last_move_end_s,
                                               "retract": retract}
                    self._save()

                self.status = f"{tag}: pulling the needle out (stage down)"   # NO A withdraw here
                self._act(f"S{i}: needle out")
                self._z(p[2])
                if c.REHOME_XY_EACH_SAMPLE:
                    self._rehome_xy(f"after S{i}")

                self._travel_to(self.home, f"{tag} -> filling station")
                self._purge_and_refill(a_start, tag)
                self._event(f"S{i}: done (purged + refilled at the filling station)")
                self.done_phases += 1

            self.status = (f"Test complete - water-cal + {n} sample(s). Needle is in the filling station."
                           if c.WATER_CAL_ENABLED else
                           f"Test complete - {n} sample(s). Needle is in the filling station.")
        except Aborted as e:
            self.error = str(e)
            self.status = f"ABORTED: {e}"
            self.link.stop_all()
        except Exception as e:
            self.error = str(e)
            self.status = f"ERROR: {e}"
            self.link.stop_all()
        finally:
            try:
                self._daq_end()
            except Exception as e:
                print(f"✗ stopping the pressure stream: {e}")
                if self.daq and self.daq.streaming:
                    try:
                        self.daq.stop_stream()
                    except Exception:
                        pass
            self._close_continuous()
            self.t_end = time.time()
            self._event("Test finished" if self.error is None else f"Test STOPPED: {self.error}")
            if self.csv_path and (self.data["watercal"] or self.data["samples"]):
                final = self.status
                self.status = "Saving CSV + making plots..."
                self._save()
                try:
                    self._save_settings()          # again, now with closed-loop + drift results
                except OSError as e:
                    print(f"✗ settings save failed: {e}")
                try:
                    self.png_paths = make_plots(self.data, self.png_dir, self.stem)
                except Exception as e:
                    print(f"✗ Plotting failed: {e}")
                self.status = final + f"  Saved {self.csv_path.name}"
                if self.daq_notes:
                    self.status += "  (DAQ warnings - see terminal)"
            if self.trial_dir is not None:
                try:
                    self._write_params_txt()
                except Exception as e:
                    print(f"✗ TEST_PARAMS txt failed: {e}")
            self.finished = True


class ReturnHome(PalpationRun):
    """After COMPLETE SETUP: drive to the saved FILLING STATION using the same safe
    travel as a test (stage down past the cup exit, XY travel, stage up into the cup).
    The injector (A) is not touched."""

    def __init__(self, link, cfg, home, samples, cup_lift_m=0):
        super().__init__(link, cfg, home, samples, daq=None, cup_lift_m=cup_lift_m)

    def run(self):
        c = self.cfg
        try:
            self.link.set_enabled(True)
            for m, us in self.travel_us.items():
                self._set_us(m, us)
            time.sleep(0.05)
            self.link.query_pos()
            self._travel_to(self.home, "Returning to the filling station")
            self.status = "At the filling station"
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


def xy_to_belts(x, y):
    """Carriage X/Y (FULL steps) -> belt counts (microsteps); inverse of belts_to_xy."""
    ms = Config.XY_MICROSTEPS
    return int(round((y - x) * ms)), int(round(-(x + y) * ms))


class HomeXY(threading.Thread):
    """X-Y homing with the two limit switches (south-west corner).
    For each axis: fast approach until the switch closes, retreat, slow
    approach (this contact point is the zero). Then zero the belt counters
    (X = 0, Y = 0 at the switch corner) and back off a little.
    Z / A are not homed (no switches). `offset` = belt counts at the zero,
    in the OLD frame, so the GUI can shift already-recorded positions."""

    def __init__(self, link, cfg, lift_z_to=None):
        super().__init__(daemon=True)
        self.link, self.cfg = link, cfg
        self.lift_z_to = lift_z_to       # drop the stage to this Z first (if known)
        self.abort_event = threading.Event()
        self.status = "Starting..."
        self.finished = False
        self.error = None
        self.offset = None

    def abort(self):
        self.abort_event.set()
        self.link.stop_all()

    def _delay(self, us):
        self.link.set_delay(1, us)
        self.link.set_delay(2, us)
        time.sleep(0.03)

    def _move_xy(self, dx, dy):
        p = list(self.link.pos)
        d1, d2 = xy_to_belts(dx, dy)
        self.link.move_abs([p[0] + d1, p[1] + d2, p[2], p[3]], self.cfg.xy_us(self.cfg.HOMING_DELAY_US),
                           self.abort_event)

    def _home_axis(self, axis):
        c = self.cfg
        toward = COREXY_DIRECTIONS["W" if axis == "X" else "S"]
        away = COREXY_DIRECTIONS["E" if axis == "X" else "N"]
        unit = (1, 0) if axis == "X" else (0, 1)
        # already on the switch? drive off it first
        if self.link.read_limits()[0 if axis == "X" else 1]:
            self.status = f"{axis}: moving off the switch"
            self._delay(c.xy_us(c.HOMING_DELAY_US))
            if not self.link.seek(away, axis, False, c.xy_steps(c.HOMING_RETREAT_STEPS * 10),
                                  c.xy_us(c.HOMING_DELAY_US), self.abort_event):
                raise Aborted(f"{axis} limit switch never released - check its wiring")
            self._move_xy(unit[0] * c.HOMING_RETREAT_STEPS, unit[1] * c.HOMING_RETREAT_STEPS)
        # one steady approach at a single slow speed; the contact point is the zero
        self.status = f"{axis}: approaching switch"
        self._delay(c.xy_us(c.HOMING_DELAY_US))
        if not self.link.seek(toward, axis, True, c.xy_steps(c.HOMING_MAX_TRAVEL_STEPS),
                              c.xy_us(c.HOMING_DELAY_US), self.abort_event):
            raise Aborted(f"{axis} limit switch not found within {c.HOMING_MAX_TRAVEL_STEPS} steps")

    def run(self):
        c = self.cfg
        try:
            self.link.set_enabled(True)
            self.link.set_soft_max(None, None)          # old frame's limits don't apply while homing
            self.link.set_travel_speeds()
            time.sleep(0.05)
            self.link.query_pos()
            if self.lift_z_to is not None:
                z = z_more_clear(self.lift_z_to, self.link.pos[2])
                if z != self.link.pos[2]:
                    self.status = "Lowering the stage to the safe height"
                    p = list(self.link.pos)
                    self.link.move_abs([p[0], p[1], z, p[3]], c.travel_delays()[3], self.abort_event)
            self._home_axis("X")
            self._home_axis("Y")
            zero_point = list(self.link.query_pos())   # switch corner, in the OLD frame
            self.link.zero_belts()
            self.offset = zero_point                   # set only once zeroing is confirmed
            self.link.query_pos()
            if c.HOMING_FINAL_BACKOFF_STEPS > 0:
                self.status = "Backing off the switches"
                self._move_xy(c.HOMING_FINAL_BACKOFF_STEPS, c.HOMING_FINAL_BACKOFF_STEPS)
            self.link.set_soft_max(c.XY_MAX_X_STEPS, c.XY_MAX_Y_STEPS)   # now in the homed frame
            self.status = "X-Y homed: X = 0, Y = 0 at the limit-switch corner"
        except Aborted as e:
            self.error = str(e)
            self.status = f"Homing stopped: {e}"
            self.link.stop_all()
        except Exception as e:
            self.error = str(e)
            self.status = f"Homing ERROR: {e}"
            self.link.stop_all()
        finally:
            self.finished = True


class Jogger:
    """Hold-to-jog. Each input source (pad button, key, ...) contributes
    {motor: sign}. Active motors get their J command re-sent every Config.JOG_REPEAT_S
    (firmware watchdog JOG_TIMEOUT_MS = 300 ms); a motor that drops out gets an H."""

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
        self.link.set_xy_ramp()
        self.link.set_delay(1, Config.xy_us(d["xy"]))      # tiers are full-step delays
        self.link.set_delay(2, Config.xy_us(d["xy"]))
        self.link.set_delay(3, Config.step_us("z", d["z"]))
        self.link.set_delay(4, Config.step_us("a", d["a"]))

    def update(self, force=False):
        desired = {1: 0, 2: 0, 3: 0, 4: 0}
        for signs in self.sources.values():
            desired.update({m: s for m, s in signs.items() if s != 0})
        now = time.time()
        resend = force or (now - self.last_send >= Config.JOG_REPEAT_S)
        for m in desired:
            if desired[m] != 0 and (resend or desired[m] != self.sent[m]):
                self.link.jog(m, desired[m])
            elif desired[m] == 0 and self.sent[m] != 0:
                self.link.halt(m)
        self.sent = desired
        if resend:
            self.last_send = now


# =============================================================================
# SECTION 9: STARTUP  (find the boards, connect, open the GUI)
# =============================================================================
class TerminalFilter:
    """Replaces sys.stdout / sys.stderr. Everything is appended to the session log file;
    it is ALSO shown in the terminal only while `show` is True (start-up) or when
    Config.TERMINAL_VERBOSE is on."""

    def __init__(self, real, logfile):
        self.real, self.logfile = real, logfile
        self.show = True
        self._lock = threading.Lock()

    def write(self, text):
        with self._lock:
            try:
                self.logfile.write(text)
                self.logfile.flush()
            except (OSError, ValueError):
                pass
            if self.show or Config.TERMINAL_VERBOSE:
                try:
                    self.real.write(text)
                    self.real.flush()
                except (OSError, ValueError):
                    pass
        return len(text)

    def log_only(self, text):
        with self._lock:
            try:
                self.logfile.write(text + "\n")
                self.logfile.flush()
            except (OSError, ValueError):
                pass

    def flush(self):
        pass

    def isatty(self):
        return False


def main():
    ap = argparse.ArgumentParser(description="Digital palpation: multi-sample needle insertion + injection")
    ap.add_argument("--motor-port", help="e.g. /dev/ttyACM0 (skips auto-discovery)")
    ap.add_argument("--daq-port", help="Teensy port, e.g. /dev/ttyACM1 (skips auto-discovery)")
    ap.add_argument("--no-daq", action="store_true", help="run motion only, no Teensy / no data")
    args = ap.parse_args()

    cfg = Config()
    Config.init_dirs()
    # session log: everything goes in here; the terminal only shows the start-up part
    log_dir = Config.DATA_DIR / Config.LOG_SUBFOLDER
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{Config.FILE_PREFIX}_log_{datetime.now():%m_%d_%Y}.txt"
    logfile = open(log_path, "a", buffering=1)
    real_out, real_err = sys.stdout, sys.stderr
    out = TerminalFilter(real_out, logfile)
    sys.stdout = sys.stderr = out
    warnings.simplefilter("ignore")                 # library warnings: not for the terminal
    out.log_only(f"\n===== session {datetime.now():%Y-%m-%d %H:%M:%S} =====")
    out.log_only("Axis speeds computed from the mechanics (1/8 microstepping):")
    out.log_only(Config.speed_table())
    try:
        _main(args, cfg, out)
    finally:
        sys.stdout, sys.stderr = real_out, real_err    # a crash still shows its error
        logfile.close()


def _main(args, cfg, out):
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
        out.show = False                     # GUI from here on: terminal stays quiet
        App(link, cfg, daq).run()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        link.close()
        if daq:
            daq.close()
        print("Motors stopped, ports closed.")


# #############################################################################
#                       GRAPHIC DESIGN  (look & feel)
# Everything below this line is about how things LOOK: the saved figures
# (SECTION 10), the window's colours / fonts / text / layout (SECTION 11) and
# the screens themselves (SECTION 12). Nothing here changes how a test runs.
# #############################################################################

# =============================================================================
# SECTION 10: PLOT STYLE  (the PNG figures saved after every test)
# =============================================================================
class PlotStyle:
    """Colours, sizes and words used in the saved figures. Colours are matplotlib
    colours: "#rrggbb" hex, or names like "black"."""

    # ---- general ----
    DPI = 100                          # resolution: pixel size = FIGSIZE (inches) x DPI
    FIRST_SAMPLE_COLOR = "#1f4fd1"     # S1 = blue ...
    LAST_SAMPLE_COLOR = "#d62728"      # ... SN = red; samples in between are blended
    WATERCAL_COLOR = "#888888"         # water-cal traces (grey) + "no data" text
    GRID_COLOR = "#d9d9d9"
    GRID_LW = 0.8                      # grid line width
    SUPTITLE_FONT_SIZE = 14            # figure title
    PANEL_TITLE_FONT_SIZE = 12         # title above each panel (encoder figure)
    LEGEND_FONT_SIZE = 9               # legends show only S1 and SN (+ "programmed")
    LEGEND_LOC = "best"
    LEGEND_FRAME = False
    RC_PARAMS = {                      # matplotlib defaults for every figure
        "axes.edgecolor": "#888888",   # axis lines
        "axes.labelcolor": "#333333",  # axis titles
        "xtick.color": "#333333",
        "ytick.color": "#333333",
        "axes.titlesize": 12,
        "axes.spines.top": False,      # no box: only the left + bottom axes
        "axes.spines.right": False,
    }

    # ---- file name endings: <FILE_PREFIX>_<date>_TRIAL_<n>_<ending>.png ----
    FILE_RECOVERED = "recovered"
    FILE_COLLAPSED = "collapsed"
    FILE_MEASURED = "measured"
    FILE_ENCODERS = "encoders"
    FILE_WATERCAL = "watercal"

    # ---- figures 1 + 2: pressure (1 x 3 panels) ----
    PRESSURE_FIGSIZE = (16, 5.4)       # inches (width, height)
    PRESSURE_LW = 1.5
    TITLE_RECOVERED = "Recovered Pressure Data"   # figure 1: programmed depth / volume
    TITLE_COLLAPSED = "Collapsed Pressure Data"   # figure 1b: same, every curve starts at 0 kPa
    COLLAPSED_YLABEL = "Pressure Change (kPa)"
    TITLE_MEASURED = "Encoder Corrected Data"     # figure 2: encoder depth / volume
    PRESSURE_YLABEL = "Pressure (kPa)"          # relative to the settling pressure (v01)
    PRESSURE_XLABELS = ("Insertion Depth (mm)", "Injection Time (s)", "Injection Radius (mm)")
    RETRACTION_LABEL = " retraction"   # vertical line at the programmed insertion depth
    RETRACTION_LINE_COLOR = "#9a9a9a"
    RETRACTION_TEXT_COLOR = "#7a7a7a"
    RETRACTION_LINE_LW = 1.5
    RETRACTION_FONT_SIZE = 9

    # ---- figure 3: encoder data (3 x 3 panels) ----
    ENCODER_FIGSIZE = (15, 9.6)
    ENCODER_TITLE = "Encoder Data"
    ENCODER_COLUMN_TITLES = ("Insertion, Z-motor", "Injection, A-motor", "Relaxation, A-motor")
    ENCODER_ROW_LABELS = ("Travel (mm)", "Rate (mm/s)", "Deviation (mm)")
    ENCODER_XLABEL = "Time (s)"
    ENC_TRAVEL_LW = 1.4                # measured lines, row 1
    ENC_RATE_LW = 1.1                  # row 2
    ENC_DEVIATION_LW = 1.2             # row 3
    PROGRAMMED_STYLE = "k--"           # the programmed profile: black dashed
    PROGRAMMED_LW = 1.6
    PROGRAMMED_LABEL = "programmed"
    ZERO_LINE_LW = 1.2                 # dashed 0 line in the Deviation row

    # ---- water-cal figure (only when WATER_CAL_ENABLED) ----
    WATERCAL_FIGSIZE = (12, 5.2)
    WATERCAL_TITLE = "Water calibration @ filling station - injection"
    WATERCAL_TITLE_FONT_SIZE = 13
    WATERCAL_XLABELS = ("time from injection command (s)", "equivalent radius r (mm) - commanded volume")
    WATERCAL_YLABEL = "pressure (kPa)"
    WATERCAL_LINE_COLOR = "#1f4fd1"
    WATERCAL_LW = 1.8


# =============================================================================
# SECTION 11: GUI STYLE  (window, fonts, colours, on-screen text, layout anchors)
# =============================================================================
class UI:
    """Window size, fonts, on-screen text and the main layout anchors.
    Coordinates are pixels from the top-left corner of the window: (x, y),
    rectangles are (x, y, width, height). Individual buttons are placed in
    App._build (SECTION 12), relative to the anchors below."""

    # ---- window ----
    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 560
    WINDOW_TITLE = "Digital Palpation v01"
    FPS = 30                           # screen refreshes per second

    # ---- fonts (pygame default font; the number is the size in pixels) ----
    FONT_HUGE = 64                     # countdown / estimated time
    FONT_BIG = 30                      # screen titles, position readout
    FONT_NORMAL = 22                   # most text + default button text
    FONT_SMALL = 18                    # hints, lists, labels
    BUTTON_RADIUS = 8                  # rounded button corners (0 = square)
    BUTTON_HOVER_LIGHTEN = 35          # button colour + this under the mouse / when held

    # ---- layout anchors ----
    #   +---------------------------+--------------------+
    #   | title / status line       | RIGHT PANEL:       |
    #   | jog buttons (X-Y pad,     | instructions,      |
    #   |   Z stage, injector)      | settings, workflow |
    #   | CURRENT POSITION panel    | buttons            |
    #   +---------------------------+--------------------+
    PANEL_X = 485                      # left edge of the right (workflow) panel
    BTN_X = 495                        # left edge of the right panel's buttons
    TEXT_X = 498                       # left edge of the right panel's text
    TEXT_W = 290                       # word-wrap width of the right panel's text
    PAD_CENTER = (150, 275)            # centre of the 8-direction X-Y jog pad
    PAD_SPACING = 64                   # distance between pad button centres
    PAD_BUTTON = 54                    # pad button size (square)
    POSITION_PANEL = (15, 425, 455, 115)   # CURRENT POSITION box (bottom left)
    KEYPAD_AREA = (15, 160, 455, 260)      # on-screen number pad (covers the jog buttons)

    # ---- on-screen text ----
    HEADER_TITLE = "DIGITAL PALPATION v01"
    WELCOME_TITLE = "Welcome to Digital Palpation!"
    WELCOME_TEXT = ("This system, developed by Brendan Unikewicz, Isaac Dobie, and Tal Cohen "
                    "is designed to help people learn more about soft materials. Before "
                    "performing any testing please be sure to read our GitHub "
                    "to learn more. Thank you!")
    HOME_TEXT = ("Bring the needle tip into the water of the cup - this is the FILLING "
                 "STATION. Set its position (X, Y, Z, and A), then select SET FILLING "
                 "STATION at the bottom of the screen.")
    CUP_TEXT = ("Now jog the Z stage DOWN until the needle tip is completely out of the "
                "cup (with a little margin above the rim), then select SET CUP EXIT. Before "
                "every X-Y move the stage drops at least this far below the filling station.")
    POS_TEXT = ("Please select the locations of each test sample. Specifically, place the "
                "needle above each sample you wish to test and record each position by "
                "selecting \"RECORD POSITION\". When complete, move to \"COMPLETE SETUP\".")
    TITLES = {                         # heading at the top of each screen's right panel
        "home": "1. Set FILLING STATION",
        "cup": "1b. Set CUP EXIT",
        "positions": "2. Sample positions",
        "returning": "Returning to the filling station...",
        "homing": "Homing X-Y...",
        "params": "3. Test settings - {n} samples + water-cal",
        "closed_loop": "3b. Closed-loop control",
        "check": "4. FINAL CHECK",
        "running": "RUNNING",
        "done": "DONE",
        "stopped": "STOPPED",
    }
    PARAM_FIELDS = [                   # key, label on the test-settings screen
        ("depth", "Insert depth mm"), ("irate", "Insert rate mm/s"),
        ("rdist", "Retract dist. mm"), ("rrate", "Retract rate mm/s"),
        ("dist", "Inject dist. mm"), ("jrate", "Inject rate mm/s")]
    CL_ROWS = [                        # key, label on the closed-loop screen
        ("tol", "Tolerance (mm)"), ("kp", "K_p"), ("ki", "K_i"), ("kd", "K_d")]


# ---- colours (R, G, B), 0..255 ----
# backgrounds
BG = (24, 24, 30)                      # window background
PANEL = (34, 34, 42)                   # right panel, position box, file list
CARD = (44, 44, 54)                    # countdown / estimated-time card
VALUE_BOX = (50, 50, 60)               # boxes behind gain / PID values
# text
TXT = (220, 220, 228)                  # normal text
DIM = (130, 130, 145)                  # hints, labels, disabled rows
GOOD_TXT = (150, 210, 150)             # live volume / flow, saved positions, cup lift OK
BUSY_TXT = (230, 200, 90)              # "RUNNING", "Homing X-Y..." headings
BIG_NUM_TXT = (120, 220, 140)          # countdown, estimated time, "DONE"
STOPPED_TXT = (230, 110, 110)          # "STOPPED"
CHECK_TXT = (240, 220, 120)            # values on the FINAL CHECK screen
ERROR_TXT = (240, 150, 120)            # messages / problems
WARN_TXT = (240, 170, 90)              # "DAQ not connected"
FOLDER_TXT = (150, 200, 240)           # folder names on the FILES screen
# buttons
GREEN = (40, 120, 70)                  # go / confirm (BEGIN, RECORD, NEXT, RUN TEST)
RED = (150, 40, 40)                    # STOP
BLUE = (40, 70, 140)                   # Z stage jog, COMPLETE SETUP
GREY = (70, 70, 78)                    # neutral buttons
TEAL = (30, 100, 110)                  # FILES, HOME X-Y, VIEW PLOTS
SEL = (60, 110, 170)                   # selected choice (syringe, SPS)
ON = (30, 140, 70)                     # a toggle that is ON (HOME X-Y once homed, keypad, CL)
INJECTOR = (140, 40, 40)               # WITHDRAW / INFUSE
NUDGE = (60, 60, 72)                   # -/+ step buttons
POWER_ON = (170, 110, 20)              # [MOTOR-POWER] POWER: ON button
PAD_HUB = (55, 55, 62)                 # dot in the middle of the X-Y pad
BTN_TEXT = (255, 255, 255)
BTN_DISABLED = (45, 45, 50)
BTN_DISABLED_TXT = (95, 95, 100)
BTN_ACTIVE_OUTLINE = (230, 230, 120)   # outline of the selected field / held button
BANNER = (160, 30, 30)                 # link-problem banner across the top
BANNER_TXT = (255, 255, 255)


# =============================================================================
# SECTION 12: GUI CODE  (buttons and screens)
# =============================================================================


def fmt_size(n):
    if n < 1024:
        return f"{n} B"
    if n < 1024 ** 2:
        return f"{n / 1024:.1f} KB"
    return f"{n / 1024 ** 2:.1f} MB"


class Btn:
    def __init__(self, rect, text, color, hover=None, font_size=None, text_color=BTN_TEXT):
        self.rect = pygame.Rect(rect)
        self.text, self.color = text, color
        self.hover = hover
        self.font = pygame.font.Font(None, font_size or UI.FONT_NORMAL)
        self.text_color = text_color
        self.enabled = True
        self.active = False       # e.g. selected field / held jog button

    def hit(self, pos):
        return self.enabled and self.rect.collidepoint(pos)

    def draw(self, screen):
        if not self.enabled:
            col, tcol = BTN_DISABLED, BTN_DISABLED_TXT
        else:
            over = self.rect.collidepoint(pygame.mouse.get_pos())
            hover = self.hover or tuple(min(255, v + UI.BUTTON_HOVER_LIGHTEN) for v in self.color)
            col = hover if (over or self.active) else self.color
            tcol = self.text_color
        pygame.draw.rect(screen, col, self.rect, border_radius=UI.BUTTON_RADIUS)
        if self.active:
            pygame.draw.rect(screen, BTN_ACTIVE_OUTLINE, self.rect, 2, border_radius=UI.BUTTON_RADIUS)
        lines = self.text.split("\n")
        h = self.font.get_linesize()
        y0 = self.rect.centery - h * len(lines) / 2 + h / 2
        for i, ln in enumerate(lines):
            surf = self.font.render(ln, True, tcol)
            screen.blit(surf, surf.get_rect(center=(self.rect.centerx, y0 + i * h)))


class App:
    # states
    S_WELCOME, S_HOME, S_POS, S_GOHOME, S_PARAMS, S_RUN, S_DONE, S_FILES, S_VIEW, S_HOMING = \
        "WELCOME", "HOME", "POSITIONS", "RETURNING HOME", "READY", "RUNNING", "DONE", "FILES", "VIEW", "HOMING X-Y"
    S_CUP, S_CHECK, S_CL = "CUP EXIT", "FINAL CHECK", "CLOSED LOOP"

    def __init__(self, link, cfg, daq=None):
        self.link, self.cfg, self.daq = link, cfg, daq
        self.jog = Jogger(link)
        pygame.display.init()                # only what we use: no audio / joystick init
        pygame.font.init()
        self.screen = pygame.display.set_mode((UI.WINDOW_WIDTH, UI.WINDOW_HEIGHT))
        pygame.display.set_caption(UI.WINDOW_TITLE)
        self.f_big = pygame.font.Font(None, UI.FONT_BIG)
        self.f_huge = pygame.font.Font(None, UI.FONT_HUGE)
        self.f = pygame.font.Font(None, UI.FONT_NORMAL)
        self.f_small = pygame.font.Font(None, UI.FONT_SMALL)
        self.clock = pygame.time.Clock()

        self.state = self.S_WELCOME
        self.prev_state = self.S_WELCOME   # where FILES returns to
        self.home = None
        self.cup_lift_m = None             # stage drop (Z microsteps) that clears the cup
        self.samples = []
        self.syringe = cfg.DEFAULT_SYRINGE
        self.cl = {k: dict(v) for k, v in cfg.CL_DEFAULTS.items()}   # closed-loop settings
        self.params = None                 # validated test settings (set by NEXT)
        self.fields = {"depth": f"{cfg.DEFAULT_INSERT_DEPTH_MM:g}",
                       "irate": f"{cfg.DEFAULT_INSERT_RATE_MM_S:g}",
                       "rdist": f"{cfg.DEFAULT_RETRACT_DIST_MM:g}",
                       "rrate": f"{cfg.DEFAULT_RETRACT_RATE_MM_S:g}",
                       "dist": f"{cfg.DEFAULT_INJECT_DIST_MM:g}",
                       "jrate": f"{cfg.DEFAULT_INJECT_RATE_MM_S:g}"}
        self.active_field = "depth"
        self.rate = cfg.DEFAULT_RATE                # ONE sample rate + gain for the whole test
        self.gain = cfg.DEFAULT_GAIN
        self.run_thread = None
        self.home_thread = None            # automatic return to HOME after COMPLETE SETUP
        self.homing_thread = None          # X-Y homing with the limit switches
        self.homing_return = None          # screen to go back to after homing
        self.xy_homed = False              # True once X/Y are relative to the switch corner
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
        self.b_homexy = Btn((20, 122, 150, 34), "HOME X-Y", TEAL, font_size=22)
        # motor power toggle: OFF cuts the current to ALL drivers right away (they cool down);
        # any jog / step / test turns them back on by itself
        self.b_power = Btn((185, 122, 120, 34), "", GREY, font_size=20)
        # [KBD-MOTOR] KEYBOARD: ON/OFF toggle button - disabled for now
        # self.b_keys = Btn((320, 78, 150, 34), "", GREY, font_size=20)

        (cx, cy), sp, sz = UI.PAD_CENTER, UI.PAD_SPACING, UI.PAD_BUTTON
        offs = {'NW': (-1, -1), 'N': (0, -1), 'NE': (1, -1), 'W': (-1, 0),
                'E': (1, 0), 'SW': (-1, 1), 'S': (0, 1), 'SE': (1, 1)}
        self.pad = {d: Btn((cx + dx * sp - sz // 2, cy + dy * sp - sz // 2, sz, sz), d, GREY)
                    for d, (dx, dy) in offs.items()}
        self.pad_center = (cx, cy)

        zi, ad = Config.Z_INSERT_SIGN, Config.INJ_DISPENSE_SIGN
        self.lin = {   # button -> (motor, sign); labels are PHYSICAL directions of the STAGE
            "Z_UP":   (Btn((300, 200, 70, 60), "▲\nUP", BLUE), (3, zi)),       # toward the needle
            "Z_DOWN": (Btn((300, 300, 70, 60), "▼\nDOWN", BLUE), (3, -zi)),
            "A_IN":   (Btn((390, 200, 70, 60), "WITHDRAW", INJECTOR, font_size=17), (4, -ad)),
            "A_OUT":  (Btn((390, 300, 70, 60), "INFUSE", INJECTOR, font_size=19), (4, ad)),
        }

        # --- single-step nudge buttons (CURRENT POSITION panel) ---
        # 2 x 2 grid: X | Y on the first row, Z | A on the second.
        self.nudge = {}       # Btn -> (axis, +1/-1)
        for axis, col, row in (("X", 0, 0), ("Y", 1, 0), ("Z", 0, 1), ("A", 1, 1)):
            bx, by = 158 + col * 222, 454 + row * 32
            mm = self.cfg.NUDGE_MM[axis.lower()]
            self.nudge[Btn((bx, by, 38, 27), f"-{mm:g}", NUDGE, font_size=18)] = (axis, -1)
            self.nudge[Btn((bx + 42, by, 38, 27), f"+{mm:g}", NUDGE, font_size=18)] = (axis, +1)

        # --- right: workflow panel (from UI.PANEL_X to the window edge) ---
        R = UI.BTN_X
        self.b_begin = Btn((R, 400, 285, 60), "BEGIN", GREEN, font_size=32)
        self.b_set_home = Btn((R, 400, 285, 60), "SET FILLING STATION", GREEN, font_size=27)
        self.b_set_cup = Btn((R, 400, 285, 60), "SET CUP EXIT", GREEN, font_size=30)
        self.b_cup_reset = Btn((R, 345, 285, 44), "RESET FILLING STATION", GREY, font_size=22)
        self.b_add = Btn((R, 360, 140, 50), "RECORD\nPOSITION", GREEN)
        self.b_undo = Btn((R + 145, 360, 140, 50), "UNDO LAST", GREY)
        self.b_rehome = Btn((R, 415, 140, 40), "RESET FILLING\nSTATION", GREY, font_size=18)
        self.b_save = Btn((R + 145, 415, 140, 40), "COMPLETE\nSETUP", BLUE, font_size=20)

        # READY screen: syringe toggle, 4 mm / mm/s fields (+ live nL / nL/s), rate + gain
        # selectors, number pad, then NEXT -> FINAL CHECK
        self.b_syr = {name: Btn((R + 78 + i * 70, 38, 66, 26), name, GREY, font_size=20)
                      for i, (name, _) in enumerate(self.cfg.SYRINGES)}
        self.field_y = {key: 68 + i * 28 for i, (key, _) in enumerate(UI.PARAM_FIELDS)}
        self.b_field = {key: Btn((R + 134, self.field_y[key], 70, 25), "", GREY, font_size=22)
                        for key, _ in UI.PARAM_FIELDS}
        # ONE SPS row + ONE gain row: the sensor streams non-stop for the whole test (v01)
        self.rate_y, self.gain_y = 242, 274
        self.b_rates = {sps: Btn((R + 92 + i * 39, self.rate_y, 36, 24), str(sps), GREY, font_size=20)
                        for i, sps in enumerate(self.cfg.SAMPLE_RATES)}
        self.b_gain = (Btn((R + 125, self.gain_y, 36, 24), "<", GREY, font_size=24),
                       Btn((R + 249, self.gain_y, 36, 24), ">", GREY, font_size=24))
        # number pad: shown over the LEFT (jog) area - big keys, and the right panel stays readable
        self.keypad = {}
        self.keypad_area = pygame.Rect(UI.KEYPAD_AREA)
        layout = [["7", "8", "9", "DEL"], ["4", "5", "6", "CLR"], ["1", "2", "3", "."], ["0"]]
        for r, row in enumerate(layout):
            for c, k in enumerate(row):
                w = 210 if k == "0" else 100
                self.keypad[k] = Btn((30 + c * 110, 170 + r * 62, w, 54), k, GREY, font_size=30)
        self.b_keypad = Btn((R, 358, 285, 27), "", GREY, font_size=22)    # show / hide the number pad
        self.b_back = Btn((R, 450, 120, 42), "BACK", GREY)
        self.b_next = Btn((R + 130, 450, 155, 42), "NEXT >", GREEN, font_size=28)
        # FINAL CHECK screen
        self.b_chk_back = Btn((R, 444, 120, 46), "BACK", GREY)
        # CLOSED-LOOP screen: per axis a toggle + tolerance / Kp / Ki / Kd steppers
        self.cl_y0 = {"z": 42, "a": 232}
        self.b_cl_toggle, self.b_cl_step = {}, {}        # axis -> Btn ; (axis, key, -1/+1) -> Btn
        for axis, y0 in self.cl_y0.items():
            self.b_cl_toggle[axis] = Btn((R, y0, 285, 32), f"Closed-Loop Control: {axis.upper()}", GREY,
                                         font_size=24)
            for r, (key, _) in enumerate(UI.CL_ROWS):
                y = y0 + 38 + r * 33
                self.b_cl_step[(axis, key, -1)] = Btn((R + 150, y, 36, 28), "-", GREY, font_size=26)
                self.b_cl_step[(axis, key, +1)] = Btn((R + 249, y, 36, 28), "+", GREY, font_size=26)
        self.b_cl_back = Btn((R, 450, 120, 42), "BACK", GREY)
        self.b_cl_next = Btn((R + 130, 450, 155, 42), "NEXT >", GREEN, font_size=28)
        self.b_start = Btn((R + 130, 444, 155, 46), "RUN TEST", GREEN, font_size=28)

        self.b_abort = Btn((R, 330, 285, 80), "STOP RUN", RED, font_size=36)
        self.b_stop_home = Btn((R, 330, 285, 80), "STOP", RED, font_size=36)

        # end-of-test screen: just three choices
        self.b_plots = Btn((R, 300, 285, 52), "VIEW PLOTS", TEAL, font_size=28)
        self.b_again = Btn((R, 362, 285, 52), "REPEAT TEST", GREEN, font_size=28)
        self.b_main = Btn((R, 424, 285, 52), "RETURN TO MAIN SCREEN", GREY, font_size=24)

        # --- files screen (full window) ---
        W = UI.WINDOW_WIDTH
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
        if s == self.S_CUP:
            return [self.b_set_cup, self.b_cup_reset]
        if s == self.S_CL:
            out = [self.b_cl_back, self.b_cl_next]
            if self.daq:                                   # closed loop needs the encoders
                out += list(self.b_cl_toggle.values()) + list(self.b_cl_step.values())
            return out
        if s == self.S_CHECK:
            return [self.b_chk_back, self.b_start]
        if s in (self.S_GOHOME, self.S_HOMING):
            return [self.b_stop_home]
        if s == self.S_POS:
            return [self.b_add, self.b_undo, self.b_rehome, self.b_save]
        if s == self.S_PARAMS:
            out = list(self.b_syr.values()) + list(self.b_field.values()) + [self.b_keypad, self.b_back, self.b_next]
            if self.show_keypad:
                out += list(self.keypad.values())
            if self.daq:
                out += list(self.b_rates.values()) + list(self.b_gain)
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
        return self.state not in (self.S_RUN, self.S_GOHOME, self.S_HOMING, self.S_FILES, self.S_VIEW)

    def _a_locked(self):
        """Injector (A) can't be moved while setting the cup exit / picking sample positions."""
        return self.state in (self.S_POS, self.S_CUP)

    # ----------------------------------------------------------- actions
    def _record_position(self):
        self.jog.release_all()
        time.sleep(0.05)
        try:
            return self.link.query_pos()
        except RuntimeError as e:
            self.msg = str(e)
            return None

    def _field_values(self):
        """Parsed test-settings fields, or None (blank / not a number)."""
        try:
            return {k: float(self.fields[k]) for k, _ in UI.PARAM_FIELDS}
        except ValueError:
            return None

    def _validate(self):
        """Check the test settings; on success store them in self.params and return True."""
        c = self.cfg
        v = self._field_values()
        if v is None:
            self.msg = "Enter a number in all the test settings"
            return False
        for key, axis, what in (("depth", "z", "Insert depth"), ("dist", "a", "Inject distance")):
            if c.mm_to_micro(axis, v[key]) < 1:
                self.msg = f"{what} must be at least {c.mm_per_micro(axis):g} mm"
                return False
        if v["rdist"] < 0 or v["rdist"] > v["depth"]:
            self.msg = "Retract distance must be 0 (none) up to the insertion depth"
            return False
        checks = [("irate", "z", "Insert rate"), ("jrate", "a", "Inject rate")]
        if v["rdist"] > 0:
            checks.append(("rrate", "z", "Retract rate"))
        for key, axis, what in checks:
            lo, hi = c.min_mm_s(axis), c.max_mm_s(axis)
            if not lo <= v[key] <= hi:
                self.msg = f"{what} must be {lo:.3g} - {hi:.3g} mm/s (quasi-static .. max)"
                return False
        self.params = v
        return True

    def _cl_step(self, axis, key, d):
        """-/+ on the closed-loop screen. Tolerance walks CL_TOL_STEPS_MM (0.01..0.5 mm);
        gains move by CL_GAIN_STEP within 0..1."""
        c, p = self.cfg, self.cl[axis]
        if key == "tol":
            steps = c.CL_TOL_STEPS_MM
            i = min(range(len(steps)), key=lambda k: abs(steps[k] - p["tol"]))
            p["tol"] = steps[max(0, min(len(steps) - 1, i + d))]
        else:
            p[key] = round(max(0.0, min(1.0, p[key] + d * c.CL_GAIN_STEP)), 2)

    def _make_run(self, daq):
        v = self.params
        return PalpationRun(self.link, self.cfg, self.home, self.samples,
                            depth_mm=v["depth"], insert_mm_s=v["irate"],
                            inject_mm=v["dist"], inject_mm_s=v["jrate"],
                            daq=daq, rate=self.rate, gain=self.gain,
                            retract_mm=v["rdist"], retract_mm_s=v["rrate"],
                            xy_homed=self.xy_homed, cup_lift_m=self.cup_lift_m or 0,
                            syringe=self.syringe, cl=self.cl)

    def _estimate_s(self):
        """Estimated test duration (s) from the current position, or None."""
        if not self.params or not self.home or not self.samples:
            return None
        return sum(self._make_run(self.daq).plan_phases(self.link.pos))

    def _start_run(self):
        if not self.link.link_ok():
            self.msg = "Can't start - motor link problem"
            return
        if self.daq and not self.daq.link_ok():
            self.msg = "Can't start - DAQ (Teensy) link problem"
            return
        if self.daq and self.daq.scale_ok() is False:
            self.msg = f"Can't start - the Teensy can't find the NAU7802 pressure ADC ({self.cfg.NAU7802_WIRING})"
            return
        bad = self._positions_out_of_bounds()
        if bad:
            self.msg = bad
            return
        if not self._validate():
            return
        self.jog.release_all()
        self.run_thread = self._make_run(self.daq)
        self.run_thread.start()
        self.state = self.S_RUN
        self.msg = ""

    def _click(self, pos):
        if self.state == self.S_PARAMS and self.show_keypad and self.keypad_area.collidepoint(pos):
            for b in self.keypad.values():          # the pad sits over the jog buttons
                if b.hit(pos):
                    self._key_digit(b.text)
            return
        if self._jog_enabled():
            if self.b_speed.hit(pos):
                self.jog.cycle_tier()
                return
            if self.b_files.hit(pos):
                self._open_files()
                return
            if self.b_homexy.hit(pos):
                self._start_homing()
                return
            if self.cfg.MOTOR_POWER_BUTTON and self.b_power.hit(pos):     # [MOTOR-POWER]
                self.jog.release_all()
                on = not self.link.enabled
                self.link.set_enabled(on)
                print("Motor power", "ON (energized)" if on else "OFF (de-energized) by the user")
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

    # Step buttons: one press moves Config.NUDGE_MM[axis] mm (whole microsteps).
    #   X / Y : carriage moves (CoreXY: one X or Y microstep = one microstep on BOTH belts)
    #   Z / A : + makes the displayed Z / A value go up
    NUDGE_BELTS = {"X": (-1, -1), "Y": (1, -1)}     # belt deltas for +X / +Y (matches E / N)

    def _past_max(self, p):
        """After homing: which max limit (if any) belt position p is beyond."""
        c = self.cfg
        if not self.xy_homed or c.XY_MAX_X_STEPS is None or c.XY_MAX_Y_STEPS is None:
            return None
        x, y = belts_to_xy(p[0], p[1])
        if x > c.XY_MAX_X_STEPS:
            return f"X {x:.0f} > max {c.XY_MAX_X_STEPS}"
        if y > c.XY_MAX_Y_STEPS:
            return f"Y {y:.0f} > max {c.XY_MAX_Y_STEPS}"
        return None

    def _positions_out_of_bounds(self):
        """Message naming the first saved position past the X/Y max, else None."""
        for name, p in ([("The FILLING STATION", self.home)] if self.home else []) + \
                       [(f"P{i}", s) for i, s in enumerate(self.samples, 1)]:
            why = self._past_max(p)
            if why:
                return f"{name} is outside the X/Y limits ({why}) - re-record it"
        return None

    def _nudge(self, axis, d):
        self.jog.release_all()
        try:
            p = self.link.query_pos(timeout=0.5)      # fresh position, not a stale one
        except RuntimeError as e:
            self.msg = str(e)
            return
        t = list(p)
        c = self.cfg
        n = c.mm_to_micro("xy" if axis in self.NUDGE_BELTS else axis.lower(), c.NUDGE_MM[axis.lower()])
        if axis in self.NUDGE_BELTS:
            b1, b2 = self.NUDGE_BELTS[axis]
            t[0] += b1 * d * n
            t[1] += b2 * d * n
            why = self._past_max(t)
            if why and not self._past_max(p):      # only block steps that cross the limit
                self.msg = f"At the X/Y limit ({why})"
                return
        else:
            t[2 if axis == "Z" else 3] += d * n
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
        elif self.homing_thread and not self.homing_thread.finished:
            self.homing_thread.abort()
        else:
            self.link.stop_all()

    def _start_homing(self):
        """HOME X-Y button: find the limit-switch corner and make it X = 0, Y = 0."""
        if not self.link.link_ok():
            self.msg = "Can't home - motor link problem"
            return
        self.jog.release_all()
        lift = None                          # drop the stage to the safe height first
        zs = ([self.home[2]] if self.home else []) + [s[2] for s in self.samples]
        if self.home and self.cup_lift_m:
            zs.append(self.home[2] - self.cfg.Z_INSERT_SIGN * self.cup_lift_m)
        for z in zs:
            lift = z if lift is None else z_more_clear(lift, z)
        self.homing_thread = HomeXY(self.link, self.cfg, lift_z_to=lift)
        self.homing_return = self.state
        self.homing_thread.start()
        self.state = self.S_HOMING

    def _finish_homing(self):
        ht = self.homing_thread
        self.jog.apply_speeds()              # back to jog speeds
        if ht.offset is not None:            # zero was set: keep saved spots physically the same
            for p in ([self.home] if self.home else []) + self.samples:
                p[0] -= ht.offset[0]
                p[1] -= ht.offset[1]
            self.xy_homed = True
        print(ht.status)
        self.msg = "X-Y homed: (0, 0) = limit-switch corner" if not ht.error else ht.status
        self.state = self.homing_return or self.S_WELCOME

    def _go_home(self):
        """After COMPLETE SETUP: drive back to the saved FILLING STATION (safe-Z lift, XY, lower)."""
        if not self.link.link_ok():
            self.msg = "Can't move - motor link problem"
            return
        bad = self._positions_out_of_bounds()
        if bad:
            self.msg = bad
            return
        self.jog.release_all()
        self.home_thread = ReturnHome(self.link, self.cfg, self.home, self.samples,
                                      cup_lift_m=self.cup_lift_m or 0)
        self.home_thread.start()
        self.state = self.S_GOHOME

    def _step_gain(self, direction):
        G = self.cfg.GAINS
        self.gain = G[max(0, min(len(G) - 1, G.index(self.gain) + direction))]

    def _state_action(self, b):
        self.msg = ""
        if b is self.b_begin:
            self.state = self.S_HOME
        elif b is self.b_set_home:
            p = self._record_position()
            if p is not None:
                self.home = p
                self.cup_lift_m = None
                self.link.board_reset = False     # fresh HOME after a reset is valid again
                self.handled_reset = False
                self.state = self.S_CUP
        elif b is self.b_set_cup:
            p = self._record_position()
            if p is not None:
                lift = (self.home[2] - p[2]) * self.cfg.Z_INSERT_SIGN     # > 0 = stage went DOWN
                if lift <= 0:
                    self.msg = ("The stage has to go DOWN (away from the needle) from the filling "
                                "station to clear the cup - jog Z DOWN, then SET CUP EXIT")
                else:
                    self.cup_lift_m = lift
                    self.state = self.S_POS
        elif b is self.b_cup_reset:
            self.home, self.cup_lift_m, self.samples = None, None, []
            self.state = self.S_HOME
        elif b is self.b_add:
            p = self._record_position()
            if p is not None:
                self.samples.append(p)
        elif b is self.b_undo and self.samples:
            self.samples.pop()
        elif b is self.b_rehome:
            self.home, self.cup_lift_m, self.samples = None, None, []
            self.state = self.S_HOME
        elif b is self.b_save:
            if not self.samples:
                self.msg = "Record at least one sample position first"
            else:
                self._go_home()
        elif b is self.b_stop_home:
            self._emergency_stop()
        elif b in self.b_field.values():
            self.active_field = next(k for k, fb in self.b_field.items() if fb is b)
        elif b is self.b_keypad:
            self.show_keypad = not self.show_keypad
        elif b in self.b_syr.values():
            self.syringe = b.text
        elif b is self.b_next:
            if self._validate():
                self.state = self.S_CL
        elif b is self.b_chk_back:
            self.state = self.S_CL
        elif b is self.b_cl_back:
            self.state = self.S_PARAMS
        elif b is self.b_cl_next:
            self.state = self.S_CHECK
        elif b in self.b_cl_toggle.values():
            axis = next(k for k, v in self.b_cl_toggle.items() if v is b)
            self.cl[axis]["on"] = not self.cl[axis]["on"]
        elif b in self.b_cl_step.values():
            axis, key, d = next(k for k, v in self.b_cl_step.items() if v is b)
            self._cl_step(axis, key, d)
        elif b in self.keypad.values():
            self._key_digit(b.text)
        elif b in self.b_rates.values():
            self.rate = next(k for k, v in self.b_rates.items() if v is b)
        elif b in self.b_gain:
            self._step_gain(-1 if b is self.b_gain[0] else +1)
        elif b is self.b_back:
            self.state = self.S_POS
        elif b is self.b_start or b is self.b_again:
            self._start_run()
        elif b is self.b_abort:
            self._emergency_stop()
        elif b is self.b_main:
            # back to the Welcome screen; BEGIN starts a fresh HOME + positions.
            # (depth / volume / rate / gains are remembered for the next setup)
            self.home, self.cup_lift_m, self.samples = None, None, []
            self.state = self.S_WELCOME
        elif b is self.b_plots:
            S = PlotStyle                     # VIEW PLOTS shows the figures in this order
            order = {S.FILE_RECOVERED: 0, S.FILE_COLLAPSED: 1, S.FILE_MEASURED: 2, S.FILE_ENCODERS: 3,
                     S.FILE_WATERCAL: 4}
            paths = sorted(self.run_thread.png_paths,
                           key=lambda q: order.get(q.stem.rsplit("_", 1)[-1], 9))
            self._open_viewer(paths, 0, self.S_DONE)
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
        cur = self.fields[f]
        if k == "CLR":
            self.fields[f] = ""
        elif k == "DEL":
            self.fields[f] = cur[:-1]
        elif len(cur) >= 8:
            return
        elif k == ".":
            if "." not in cur:
                self.fields[f] = (cur or "0") + "."
        elif k.isdigit():
            self.fields[f] = k if cur == "0" else cur + k

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
        if not (20 <= x <= UI.WINDOW_WIDTH - 85):
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
                bw, bh = UI.WINDOW_WIDTH - 20, 455
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
            if event.unicode.isdigit() or event.unicode == ".":
                self._key_digit(event.unicode)
                return
            if k == pygame.K_BACKSPACE:
                self._key_digit("DEL")
                return
            if k == pygame.K_TAB:
                keys = [key for key, _ in UI.PARAM_FIELDS]
                self.active_field = keys[(keys.index(self.active_field) + 1) % len(keys)]
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

    def _mm(self, p):
        """Raw firmware counts -> (X, Y, Z, A) in mm."""
        c = self.cfg
        x, y = belts_to_xy(p[0], p[1])                      # full steps
        return (x * c.mm_per_full("xy"), y * c.mm_per_full("xy"),
                p[2] * c.mm_per_micro("z"), p[3] * c.mm_per_micro("a"))

    def _fmt(self, p):
        x, y, z, _ = self._mm(p)
        return f"X{x:+.1f} Y{y:+.1f} Z{z:+.2f} mm"

    def _draw_files(self):
        sc = self.screen
        W = UI.WINDOW_WIDTH
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
                    self._text(f"[DIR]  {p.name}/", (30, y + 6), self.f, FOLDER_TXT)
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
        W = UI.WINDOW_WIDTH
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
            pygame.draw.rect(sc, PANEL, (UI.PANEL_X, 0, UI.WINDOW_WIDTH - UI.PANEL_X, UI.WINDOW_HEIGHT))
            self._text(UI.HEADER_TITLE, (20, 14), self.f_big)
            daq_txt = "DAQ: connected" if self.daq else "DAQ: NOT CONNECTED (motion only, no data)"
            self._text(f"step: {self.state}   |   {daq_txt}", (20, 44), self.f_small,
                       DIM if self.daq else WARN_TXT)

            # jog panel
            en = self._jog_enabled()
            name, color, d = self.jog.tier_info()
            self.b_speed.text = f"JOG: {name}"
            self.b_speed.color = color
            # [KBD-MOTOR] KEYBOARD toggle button - disabled for now
            # self.b_keys.text = f"KEYBOARD: {'ON' if self.keyboard_on else 'OFF'}"
            # self.b_keys.color = ON if self.keyboard_on else GREY
            # (and add self.b_keys back into the list just below)
            self.b_homexy.text = "HOME X-Y"                      # same label always; green = homed
            self.b_homexy.color = ON if self.xy_homed else TEAL
            self.b_power.text = "POWER: ON" if self.link.enabled else "POWER: OFF"
            self.b_power.color = POWER_ON if self.link.enabled else GREY
            power_btn = [self.b_power] if self.cfg.MOTOR_POWER_BUTTON else []   # [MOTOR-POWER]
            for b in ([self.b_speed, self.b_files, self.b_homexy] + power_btn
                      + list(self.pad.values()) + [v[0] for v in self.lin.values()]):
                b.enabled = en
                if b is self.b_speed and self.state == self.S_POS:
                    b.enabled = False             # fixed to FINE while recording sample positions
                if self._a_locked() and (b is self.lin["A_IN"][0] or b is self.lin["A_OUT"][0]):
                    b.enabled = False
                b.draw(sc)
            pygame.draw.circle(sc, PAD_HUB, self.pad_center, 18)
            self._text("CORE X,Y", (150, 175), self.f_small, DIM, center=True)
            self._text("Z STAGE", (335, 185), self.f_small, DIM, center=True)
            self._text("INJECTOR", (425, 185), self.f_small, DIM, center=True)

            # live position
            p = self.link.pos
            x, y = belts_to_xy(p[0], p[1])
            pygame.draw.rect(sc, PANEL, UI.POSITION_PANEL, border_radius=8)
            self._text("CURRENT POSITION (mm)", (28, 435), self.f_small, DIM)
            xm, ym, zm, am = self._mm(p)
            self._text(f"X {xm:+.2f}", (28, 457), self.f_big)
            self._text(f"Y {ym:+.2f}", (250, 457), self.f_big)
            self._text(f"Z {zm:+.3f}", (28, 489), self.f_big)
            self._text(f"A {am:+.3f}", (250, 489), self.f_big)
            for b, (axis, _) in self.nudge.items():
                b.enabled = en and not (axis == "A" and self._a_locked())
                b.draw(sc)
            self._draw_workflow()

        for b in self._state_buttons():
            b.draw(sc)
        if self.msg:
            if self.state in (self.S_FILES, self.S_VIEW):
                self._text(self.msg, (20, 470), self.f_small, ERROR_TXT)
            else:
                self._wrap(self.msg, (498, 500), 295, max_lines=3, font=self.f_small, line_h=16,
                           color=ERROR_TXT)

        # link-health banner (always on top)
        warn = self.link.health_text() or (self.daq.health_text() if self.daq else None)
        if warn:
            pygame.draw.rect(sc, BANNER, (0, 0, UI.WINDOW_WIDTH, 26))
            self._text(warn, (UI.WINDOW_WIDTH // 2, 13), self.f, BANNER_TXT, center=True)
        pygame.display.flip()

    def _draw_workflow(self):
        sc = self.screen
        c = Config
        R, W = UI.TEXT_X, UI.TEXT_W
        T = UI.TITLES
        st = self.state
        if st == self.S_WELCOME:
            y = self._wrap(UI.WELCOME_TITLE, (R, 30), W, font=self.f_big, line_h=30)
            self._wrap(UI.WELCOME_TEXT, (R, y + 16), W, max_lines=14)
        elif st == self.S_HOME:
            self._text(T["home"], (R, 30), self.f_big)
            self._wrap(UI.HOME_TEXT, (R, 70), W)
        elif st == self.S_CUP:
            self._text(T["cup"], (R, 24), self.f_big)
            y = self._wrap(UI.CUP_TEXT, (R, 60), W, font=self.f_small, line_h=17)
            mpz = c.mm_per_micro("z")
            lift = (self.home[2] - self.link.pos[2]) * c.Z_INSERT_SIGN * mpz
            self._text(f"Filling station Z: {self.home[2] * mpz:+.3f} mm", (R, y + 14), self.f)
            self._text(f"Stage now Z:       {self.link.pos[2] * mpz:+.3f} mm", (R, y + 38), self.f)
            self._text(f"Cup lift = {lift:.3f} mm", (R, y + 68), self.f_big,
                       GOOD_TXT if lift > 0 else ERROR_TXT)
        elif st == self.S_POS:
            self._text(T["positions"], (R, 18), self.f_big)
            y = self._wrap(UI.POS_TEXT, (R, 50), W, font=self.f_small, line_h=17)
            y += 8
            self._text(f"FILL {self._fmt(self.home)}", (R, y), self.f_small, GOOD_TXT)
            y += 18
            if self.cup_lift_m:
                self._text(f"cup lift {self.cup_lift_m * c.mm_per_micro('z'):.2f} mm", (R + 38, y),
                           self.f_small, GOOD_TXT)
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
            y = self._wrap(T["returning"], (R, 30), W, font=self.f_big,
                           line_h=28, color=BUSY_TXT)
            if ht:
                self._wrap(ht.status, (R, y + 10), 285)
        elif st == self.S_HOMING:
            ht = self.homing_thread
            self._text(T["homing"], (R, 30), self.f_big, BUSY_TXT)
            if ht:
                self._wrap(ht.status, (R, 70), 285)
        elif st == self.S_PARAMS:
            self._text(T["params"].format(n=len(self.samples)), (R, 12), self.f)
            self._text("Syringe", (R, 44), self.f)
            for name, b in self.b_syr.items():
                b.color = SEL if name == self.syringe else GREY
                b.active = name == self.syringe
            for key, label in UI.PARAM_FIELDS:
                y = self.field_y[key]
                self._text(label, (R, y + 6), self.f)
                b = self.b_field[key]
                b.text = self.fields[key] or "0"
                b.active = (key == self.active_field)
            # live volume (dist) and flow rate (jrate) for the selected syringe
            v = self._field_values() or {}
            nlmm = c.nl_per_mm(self.syringe)
            for key, txt in (("dist", fmt_volume(nlmm * v["dist"]) if "dist" in v else "-"),
                             ("jrate", fmt_flow(nlmm * v["jrate"]) if "jrate" in v else "-")):
                self._text(txt, (R + 210, self.field_y[key] + 6), self.f_small, GOOD_TXT)
            if self.daq:
                self._text("SPS", (R, self.rate_y + 5), self.f)
                for sps, b in self.b_rates.items():
                    b.color = SEL if sps == self.rate else GREY
                    b.active = sps == self.rate
                y = self.gain_y
                self._text("Gain", (R, y + 4), self.f)
                pygame.draw.rect(sc, VALUE_BOX, (R + 165, y, 80, 24), border_radius=6)
                self._text(f"x{self.gain}", (R + 205, y + 12), self.f, TXT, center=True)
                minus, plus = self.b_gain
                minus.enabled = self.gain != c.GAINS[0]
                plus.enabled = self.gain != c.GAINS[-1]
                self._text(f"one setting for the whole test ({c.DAQ_WARMUP_S:g} s warm-up first)",
                           (R, y + 32), self.f_small, DIM)
            else:
                self._text("DAQ not connected:", (R, 244), self.f, WARN_TXT)
                self._text("motion runs, nothing is recorded", (R, 268), self.f_small, DIM)
            if self.show_keypad:                  # number pad covers the jog area
                pygame.draw.rect(sc, PANEL, self.keypad_area, border_radius=10)
                self._text(f"typing: {dict(UI.PARAM_FIELDS)[self.active_field]}", (252, 377), self.f_small, DIM)
            self.b_keypad.text = f"DIGITAL KEYBOARD: {'ON' if self.show_keypad else 'OFF'}"
            self.b_keypad.color = ON if self.show_keypad else GREY
        elif st == self.S_CHECK:
            self._draw_check()
        elif st == self.S_CL:
            self._draw_cl()
        elif st == self.S_RUN:
            rt = self.run_thread
            self._text(T["running"], (R, 20), self.f_big, BUSY_TXT)
            if rt.warming:
                self._text("pressure sensor warm-up", (R, 52), self.f)
            elif rt.done_phases == 0 and c.WATER_CAL_ENABLED and rt.phase_idx == 0:
                self._text("water-cal (filling station)", (R, 52), self.f)
            else:
                self._text(f"sample {rt.phase_idx} of {len(rt.samples)}", (R, 52), self.f)
            # countdown to the estimated end of the test (re-aimed at every sample)
            pygame.draw.rect(sc, CARD, (R - 3, 80, 291, 92), border_radius=10)
            rem = rt.remaining_s()
            self._text("--:--:--" if rem is None else fmt_hms(rem), (R + 142, 118), self.f_huge,
                       BIG_NUM_TXT, center=True)
            self._text("time remaining (estimated)", (R + 142, 158), self.f_small, DIM, center=True)
            self._wrap(rt.status, (R, 184), 285, max_lines=6)
        elif st == self.S_DONE:
            rt = self.run_thread
            ok = rt and rt.error is None
            self._text(T["done"] if ok else T["stopped"], (R, 30), self.f_big,
                       BIG_NUM_TXT if ok else STOPPED_TXT)
            if rt:
                self._wrap(rt.status, (R, 70), 285, max_lines=9)
            # greyed out when the test made no plots (no DAQ / matplotlib / stopped early)
            self.b_plots.enabled = bool(rt and rt.png_paths)

    def _draw_cl(self):
        """CLOSED-LOOP screen: Z and A each get a toggle + tolerance / Kp / Ki / Kd."""
        R = UI.TEXT_X
        self._text(UI.TITLES["closed_loop"], (R, 12), self.f)
        if not self.daq:
            self._wrap("Closed-loop control needs the Teensy DAQ (it reads the encoders). "
                       "Not connected - the test runs open-loop.", (R, 60), 285, color=WARN_TXT)
            return
        for axis, y0 in self.cl_y0.items():
            p = self.cl[axis]
            tb = self.b_cl_toggle[axis]
            tb.color = ON if p["on"] else GREY
            tb.active = p["on"]
            for r, (key, label) in enumerate(UI.CL_ROWS):
                y = y0 + 38 + r * 33
                col = TXT if p["on"] else DIM
                self._text(label, (R, y + 6), self.f, col)
                pygame.draw.rect(self.screen, VALUE_BOX, (R + 190, y, 56, 28), border_radius=6)
                val = f"{p[key]:.3g}" if key == "tol" else f"{p[key]:.2f}"
                self._text(val, (R + 218, y + 14), self.f, col, center=True)
                lo, hi = (self.cfg.CL_TOL_STEPS_MM[0], self.cfg.CL_TOL_STEPS_MM[-1]) if key == "tol" else (0.0, 1.0)
                self.b_cl_step[(axis, key, -1)].enabled = p["on"] and p[key] > lo + 1e-9
                self.b_cl_step[(axis, key, +1)].enabled = p["on"] and p[key] < hi - 1e-9
            # speed headroom: closed loop can only speed up to the axis maximum
            rate = (self.params or {}).get("irate" if axis == "z" else "jrate")
            if rate:
                mx = self.cfg.max_mm_s(axis)
                room = mx / rate
                y = y0 + 38 + 4 * 33 + 2
                txt = f"rate {rate:g} of max {mx:.3g} mm/s"
                if room < 1.05:
                    self._text(txt + " - can't speed up", (R, y), self.f_small, ERROR_TXT)
                elif room < 1.5:
                    self._text(txt + f" - speed-up up to x{room:.2f}", (R, y), self.f_small, BUSY_TXT)
                else:
                    self._text(txt, (R, y), self.f_small, DIM)

    def _draw_check(self):
        """FINAL CHECK: every user-selected option + the estimated test time."""
        c, R = Config, UI.TEXT_X
        v = self.params
        nlmm = c.nl_per_mm(self.syringe)
        hi, lab = CHECK_TXT, DIM
        self._text(UI.TITLES["check"], (R, 12), self.f_big)
        mpz = c.mm_per_micro("z")
        tr = c.travel_delays()
        rows = [
            ("Samples", f"{len(self.samples)}" + (" + water-cal" if c.WATER_CAL_ENABLED else "")),
            ("Syringe", f"{self.syringe}  (ID {c.syringe_id(self.syringe):g} mm)"),
            ("Insert", f"{v['depth']:g} mm @ {v['irate']:g} mm/s"),
            ("Retract", f"{v['rdist']:g} mm @ {v['rrate']:g} mm/s" if v["rdist"] > 0 else "none"),
            ("Inject", f"{v['dist']:g} mm @ {v['jrate']:g} mm/s"),
            ("Volume", f"{fmt_volume(nlmm * v['dist'])}  @ {fmt_flow(nlmm * v['jrate'])}"),
            ("Relax hold", f"{c.RELAX_HOLD_S:g} s"),
            ("Purge", f"{c.PURGE_MM:g} mm  ({fmt_volume(nlmm * c.PURGE_MM)})"),
            ("Cup lift", f"{(self.cup_lift_m or 0) * mpz:.2f} mm"),
            ("Travel", f"FINE  XY {c.mm_s_for_us('xy', tr[1]):.3g}, Z {c.mm_s_for_us('z', tr[3]):.3g} mm/s"),
            ("DAQ", f"{self.rate} SPS, gain x{self.gain}, warm-up {c.DAQ_WARMUP_S:g} s" if self.daq
             else "NOT CONNECTED - no data"),
            ("X-Y homed", "yes" if self.xy_homed else "no - homes at the start"),
            ("Re-home", "X-Y after every sample" if c.REHOME_XY_EACH_SAMPLE else "off"),
        ] + [
            (f"Closed {ax.upper()}", (f"tol {p['tol']:g} mm  Kp {p['kp']:g}  Ki {p['ki']:g}  Kd {p['kd']:g}"
                                      + (" (max speed)" if v[('irate' if ax == 'z' else 'jrate')]
                                         >= 0.95 * c.max_mm_s(ax) else "")
                                      if p["on"] and self.daq else "off (open loop)"))
            for ax, p in self.cl.items()
        ]
        y = 44
        for name, val in rows:
            self._text(name, (R, y), self.f_small, lab)
            font = self.f_small if name.startswith("Closed") else self.f
            self._text(val, (R + 78, y - (0 if font is self.f_small else 2)), font,
                       hi if name not in ("DAQ", "X-Y homed") or
                       (self.daq and name == "DAQ") or (self.xy_homed and name == "X-Y homed")
                       else ERROR_TXT)
            y += 20
        est = self._estimate_s()
        pygame.draw.rect(self.screen, CARD, (R - 3, y + 6, 291, 70), border_radius=10)
        self._text("ESTIMATED TEST TIME", (R + 142, y + 20), self.f_small, DIM, center=True)
        self._text("--:--:--" if est is None else fmt_hms(est), (R + 142, y + 50), self.f_huge,
                   BIG_NUM_TXT, center=True)

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
        is meaningless - throw them away and send the user back to SET FILLING STATION."""
        if self.link.board_reset and not self.handled_reset:
            self.handled_reset = True
            self.jog.release_all()
            self.home, self.cup_lift_m, self.samples = None, None, []
            self.xy_homed = False            # counters restarted at 0 - zero is lost
            if self.state == self.S_HOMING:
                self.homing_return = self.S_HOME
            if self.state not in (self.S_RUN, self.S_HOMING, self.S_FILES, self.S_VIEW):   # incl. CUP / CHECK
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
        if self.homing_thread and not self.homing_thread.finished:
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
            if self.state == self.S_POS and self.jog.tier != 0:    # sample-position setup = FINE only
                self.jog.release_all()
                self.jog.tier = 0
                self.jog.apply_speeds()
            if self._jog_enabled():
                self.jog.update()
            self._auto_motor_off()
            if self.state == self.S_HOMING and self.homing_thread.finished:
                self._finish_homing()
            if self.state == self.S_GOHOME and self.home_thread.finished:
                self.jog.apply_speeds()           # back to jog speeds
                self.state = self.S_PARAMS
                if self.home_thread.error:
                    self.msg = f"Return to the filling station stopped ({self.home_thread.error})"
            if self.state == self.S_RUN and self.run_thread.finished:
                rt = self.run_thread
                d1, d2 = rt.xy_offset_total               # X-Y re-homing moved the frame:
                if (d1 or d2) and self.home:              # keep the saved spots physically the same
                    for p in [self.home] + self.samples:
                        p[0] -= d1
                        p[1] -= d2
                if rt.xy_homed:
                    self.xy_homed = True
                self.state = self.S_DONE if self.home else self.S_HOME
                self.jog.apply_speeds()       # back to jog speeds after the test
            self._draw()
            self.clock.tick(UI.FPS)

        self._emergency_stop()
        pygame.quit()


if __name__ == "__main__":
    main()


# =============================================================================
# REFERENCE NOTES
# =============================================================================
# Workflow
# --------
#   1. Run:  ./DigitalPalpation_v01.py
#            (optional: --motor-port /dev/ttyACM0 --daq-port /dev/ttyACM1, --no-daq)
#      The terminal prints the speed table computed from the mechanics.
#   2. Welcome screen                                         -> BEGIN
#   3. Jog X/Y/Z (and the injector A) so the needle tip sits IN the water of the
#      filling-station cup                                    -> SET FILLING STATION
#   4. Jog the Z stage DOWN until the needle is fully out of the cup
#                                                             -> SET CUP EXIT
#      (that distance = the CUP LIFT: before ANY X-Y move the stage drops at least
#       this far below the filling station, and below every sample height)
#   5. Jog to directly ABOVE sample 1                         -> RECORD POSITION
#      ... repeat for samples 2, 3, ... N                    -> COMPLETE SETUP
#      (jog fixed to FINE; the injector is LOCKED on the cup-exit + positions screens)
#   6. The needle drives back into the filling station by itself. Test-settings
#      screen: syringe (10 / 50 / 250 uL), insertion depth + rate, injection
#      distance + rate (live: injected volume in nL / uL and flow in nL/s),
#      sample rate, insertion + injection gain                -> NEXT
#   7. FINAL CHECK: every setting + the estimated test time (hh:mm:ss) -> RUN TEST
#      (v01) The test starts with the PRESSURE SENSOR WARM-UP: rate + gain are set and the
#      ADC calibrated ONCE, then DAQ_WARMUP_S (10 s) with nothing moving; the mean of its
#      last SETTLE_WINDOW_S = the SETTLING PRESSURE = 0 kPa for every kPa value of the test
#      (nothing else is zeroed). The sensor then streams non-stop until the test
#      ends; each recorded phase below is cut out of that one stream.
#   8. RUN TEST (countdown hh:mm:ss to the estimated end, re-aimed at every sample):
#         all moves between positions run at the FINE jog speed (TRAVEL_JOG_TIER)
#         go to the FILLING STATION (stage down to safe height, XY, stage up into the cup)
#         WATER-CAL in the cup  [recorded: injection]
#             inject the injection distance, purge PURGE_MM (2 mm), withdraw back to
#             the starting plunger level (refill)
#         for every sample i = 1..N:
#             travel to sample i  (stage down to safe height, XY, stage up to saved Z)
#             insert   : stage UP by the insertion depth at the insertion rate
#                                                       [recorded: insertion, insertion gain]
#             inject   : A infuses the injection distance at the injection rate
#                                                       [recorded: injection, injection gain]
#             hold     : RELAX_HOLD_S (5 s), needle stays in [recorded: relaxation]
#             pull out : stage back DOWN to the saved height - the injector does NOT withdraw
#             travel to the FILLING STATION (needle goes into the cup)
#             purge PURGE_MM (2 mm), THEN withdraw back to the starting plunger level
#         The injector never withdraws at a sample - only in the filling station.
#         save CSV (after every sample, so a crash never loses finished samples)
#         make the quick-view plots
#   After every sample (and once at the start if X-Y wasn't homed): the stage drops to
#   the safe height and X-Y is RE-HOMED on the limit switches; every saved position is
#   shifted with it, so X-Y drift can't build up (Config.REHOME_XY_EACH_SAMPLE). The drift
#   found at each re-home is printed and saved in settings.json ("results").
#   CLOSED LOOP (screen 3b, needs the Teensy): per axis Z / A a toggle, a tolerance
#   (0.01-0.5 mm) and Kp / Ki / Kd (0-1; integral-only by default). During insertion (Z)
#   and injection (A) the encoder travel is compared with the programmed trajectory every
#   ~25 ms; outside the tolerance the move is sped up / slowed down (firmware "V us",
#   0.5x-2x), and a shortfall left at the end (skipped microsteps) is made up with extra
#   steps in the same direction - never backwards. See Config.CL_* for the details.
#   STOP RUN (or SPACE) halts every motor immediately and aborts the run.
#
# X-Y homing (HOME X-Y button, any time except during a test):
#   Limit switches (normally open, to GND) sit at the SOUTH-WEST corner:
#   X switch -> CNC shield X endstop header (D9), Y switch -> Y header (D10).
#   Homing first drops the Z stage to the safe height (if positions exist),
#   then for X and then Y: one steady approach to the switch at HOMING_DELAY_US
#   (1.1x slower than the FINE jog). That contact point becomes X = 0, Y = 0
#   (within 1 step at this speed); the carriage then backs
#   off HOMING_FINAL_BACKOFF_STEPS in +X and +Y. Already-recorded HOME / sample
#   positions are shifted into the new frame, so they still point at the same
#   physical spots. Manual jogs can't drive into a pressed switch. Z and A have
#   no switches and are not homed.
#   The HOME X-Y button turns green once homed (its label doesn't change).
#   After homing, XY_MAX_X_STEPS / XY_MAX_Y_STEPS are sent to the firmware:
#   jogs stop at them, +1 steps and moves past them are refused, and a test /
#   return-to-HOME won't start if a saved position lies outside them.
#
# Motor power [MOTOR-POWER]: CURRENTLY the motors are ALWAYS energized - no POWER
# button and no idle power-off (this Arduino's D8 enable pin is shorted to GND; the
# firmware is built with ENABLE_PIN_CONTROL 0 and never drives an enable pin).
# The feature is still in both codes. To bring it back after swapping hardware:
#   firmware: ENABLE_PIN_CONTROL 1, ENABLE_PIN 8 (new Arduino, NO EN-GND jumper on
#             the shield!) or 11 (wire D11 -> EN header through ~330 ohm), re-flash
#   here:     Config.MOTOR_POWER_BUTTON = True, Config.AUTO_MOTOR_OFF = True
# Then all 4 motors power on/off together; the button cuts them immediately (not during
# a test), idle-off after MOTOR_IDLE_OFF_S, any jog / step / test re-energizes.
#
# Keyboard: SPACE = stop, ESC = quit, typing numbers on the test-settings screen.
#   Moving the motors from the keyboard is currently DISABLED (commented out).
#   To bring it back, uncomment every block tagged [KBD-MOTOR]. It was:
#   arrows = X/Y   W = WITHDRAW, S = INFUSE   R = Z up, F = Z down   T = jog speed
#
# Units, mechanics and microstepping (all four drivers at 1/8: jumpers M0 + M1, M2 open)
#   Everything on screen and in the CSV is in mm / mm/s. The firmware counts
#   MICROSTEPS; the program converts using SECTION 2:
#     X/Y : GT2 belt (2 mm) x 20-tooth pulley (ASSUMED) = 40 mm/rev -> 0.2 mm/full step
#     Z   : Tr8x8 lead screw, 8 mm/rev                          -> 0.04 mm/full step
#     A   : 1 mm/rev actuator                                   -> 0.005 mm/full step
#   Max speed = 1 microstep per MIN_STEP_US (188 us, the old fastest level 3);
#   quasi-static minimum = QUASI_STATIC_FULLSTEPS_S (3 full steps/s).
#     axis  mm/microstep  quasi-static   FAST (max)
#     X/Y   0.025         0.60 mm/s      ~133 mm/s
#     Z     0.005         0.12 mm/s      ~26.6 mm/s
#     A     0.000625      0.015 mm/s     ~3.32 mm/s
#   -/+ buttons move NUDGE_MM (0.1 mm) per press = 4 XY, 20 Z, 160 A microsteps.
#   If your XY pulleys aren't 20-tooth, change XY_PULLEY_TEETH (check: jog 50 mm
#   on the readout and measure).
#
# Data files  (~/digital_palpation/<MM_DD_YYYY>/TRIAL_<n>/...)
# ---------------------------------------------------------------
#   Every RUN TEST / REPEAT TEST makes the next TRIAL_<n> folder of that day (1, 2, 3, ...).
#   CSV_FILES/DP_<MM_DD_YYYY>_TRIAL_<n>.csv             one wide CSV per test
#   CSV_FILES/DP_<MM_DD_YYYY>_TRIAL_<n>_settings.json   positions, gains, syringe, ...
#   CSV_FILES/DP_<MM_DD_YYYY>_TRIAL_<n>_continuous.csv  (v01) EVERY sample of the test:
#        TIME (s) since the sensor started | ACTIVITY (warm-up, S2 travel, S2 insertion,
#        S2 injection, S2 injection: relaxation, S2 purge, ...) | PRESSURE (bits) |
#        PRESSURE (kPa, re settling) | Z-ENCODER | A-ENCODER
#   PNG_FILES/DP_<...>_recovered.png  FIGURE 1 "Recovered Pressure Data" (programmed depth /
#                                     volume): pressure (kPa) vs insertion
#                                     depth | vs time (injection + relaxation) | vs radius
#                                     of the commanded injected volume (V = 4/3 pi r^3)
#   PNG_FILES/DP_<...>_collapsed.png  FIGURE 1b "Collapsed Pressure Data": figure 1 with each
#                                     insertion curve minus its first value and each injection
#                                     curve minus its first value -> all start at 0 kPa
#   PNG_FILES/DP_<...>_measured.png   FIGURE 2 (encoders): the same three, with depth and
#                                     volume from the AS5600 encoders and time from the
#                                     measured plunger start
#                                     (both: sample 1 blue -> sample N red; legend S1 + SN)
#   PNG_FILES/DP_<...>_encoders.png   encoders in mm vs the programmed profile: travel,
#                                     rate, measured - programmed; Z insertion | A
#                                     injection | A hold (numbers printed in the terminal)
#   PNG_FILES/DP_<...>_watercal.png   water-cal pressure vs time | vs radius
#   Pressure (kPa) = bits / 2^23 x (0.5 x 3.3 V / gain) / (0.2584 mV/V/psi x 3.3 V) x 6.894757,
#   minus the SETTLING PRESSURE (end of the warm-up) - no other zeroing. SECTION 2 holds the numbers.
#   CSV columns, left to right (blank cells pad shorter columns):
#     WATERCAL_INJECTION-TIME (ms) | -DISTANCE (mm) | -PRESSURE (bits) | -PRESSURE (kPa)
#        | -Z-ENCODER (counts|V) | -A-ENCODER (counts|V)
#     (v01: every PRESSURE (kPa) column is relative to the SETTLING PRESSURE - no pre-roll zero)
#     S1_INSERTION-TIME (ms) | -DEPTH (mm) | -PRESSURE (bits) | -PRESSURE (kPa) | -Z-ENCODER | -A-ENCODER
#     S1_INJECTION-TIME (ms) | -DISTANCE (mm) | -PRESSURE (bits) | -PRESSURE (kPa) | -Z-ENCODER | -A-ENCODER
#     S1_RELAXATION-TIME (ms) | -PRESSURE (bits) | -PRESSURE (kPa) | -Z-ENCODER | -A-ENCODER
#     S2_... (same 17 columns) ... SN_...
#   ENCODER = AS5600, read by the Teensy at the same instant as each pressure sample.
#   ONE shaft turn = 0..4095 counts (I2C mode, column unit "counts"; blank cell = a
#   failed read) or 0 -> 3.3 V (analog mode, unit "V"); then it wraps back to 0 (a
#   saw-tooth over multi-turn moves). The mode is set in the TEENSY firmware
#   (ENCODER_MODE); the script asks the Teensy at start-up and labels the CSV to match.
#   I2C wiring (3.3 V ONLY - the Teensy 4.1 is not 5 V tolerant), one encoder per bus:
#     Z encoder -> Wire1: SDA pin 17, SCL pin 16      A encoder -> Wire2: SDA pin 25, SCL pin 24
#     NAU7802 stays on Wire: SDA 18, SCL 19.   OUT pins disconnected, DIR tied as before.
#   Cables > 50 cm: firmware runs these buses at 100 kHz, retries a failed read once and
#   re-initialises a bus after 20 failures in a row; "?" reports enc_err=<A>/<Z> counts.
#   Every phase's TIME restarts at 0 ms when its motor move starts (relaxation:
#   0 ms when the injection ends). The PRE_ROLL_S baseline recorded just before
#   each move shows up as a few rows with NEGATIVE time and 0 steps; insertion and
#   water-cal also keep recording POST_ROLL_S after the move (rows at full depth).
#
# How STEPS are matched to pressure samples (no extra wiring)
# -----------------------------------------------------------
#   The Teensy timestamps every ADC conversion with its own microsecond clock and
#   streams it live. Z and A step on a fixed cadence (no ramp), so a move of N
#   microsteps takes exactly N x step-delay. The host knows when it told the motor
#   to start, so mm(t) = mm_per_microstep x (t - move_start) / step_delay, clamped
#   to 0..N microsteps. Expected accuracy: a few microsteps (USB command latency).
#
# Units / position memory: positions are absolute microstep counts kept by the
# motor firmware since the serial port was opened (opening the port resets the
# Arduino -> all counters start at 0), so the filling station / sample positions
# are only valid while this program stays open (X/Y are re-anchored by HOME X-Y).
# Pressure is raw ADC bits (no kPa calibration yet). The motors are open-loop;
# the AS5600 encoders are recorded so skipped steps can be spotted afterwards.
