#!/usr/bin/env python3
"""
Combined Motor Control + Teensy Data Logging with CRC16 Verification
Modular structure for easy editing and maintenance

From terminal to make script executable: chmod +x /home/nsm1/Python_Scripts/PI_MASTER_UIUX.py

.. but keep in mind the name and location of the file may change

create a desktop entry file nano ~/Desktop/basic-control-v01.desktop

add contents:

[Desktop Entry]
Name=Basic Control
Comment=Teensy Motor Control and Data Logger
Exec=/home/nsm1/Python_Scripts/PI_MASTER_UIUX.py
Icon=/home/nsm1/Python_Scripts/motor-icon.png
Terminal=false
Type=Application
Categories=Development;
StartupNotify=true

now, make the desktop entry executable: chmod +x ~/Desktop/basic-control-v01.desktop

create a wrapper shell, run_basic_control_v01.sh:

#!/bin/bash
cd /home/nsm1/Python_Scripts
python3 PI_MASTER_UIUX.py

make it executable: chmod +x run-basic-control-v01.sh

"""
import serial
import serial.tools.list_ports
import time
import pygame
import struct
import argparse
from datetime import datetime
from pathlib import Path
import os
import sys
import subprocess

# ================= CONFIGURATION MODULE =================
class Config:
    """Central configuration for all hardware and logging parameters"""
    # Serial ports - filled in automatically by discover_devices() at startup.
    # Can be overridden with --motor-port / --data-port on the command line.
    MOTOR_PORT = None
    DATA_PORT = None
    BAUD = 115200

    # Device discovery
    IDENTIFY_CMD = b'I\n'
    BOOT_WAIT = 2.2       # seconds to let a board reset-on-open finish booting
    IDENTIFY_TIMEOUT = 1.5  # seconds to wait for an ID reply once probed

    # Logging
    HOME_DIR = Path.home()
    LOG_DIR = HOME_DIR / "teensy_logs"
    LOG_DURATION = 5  # seconds - must match LOG_DURATION_MS in the Teensy sketch
    PACKET_SIZE = 13  # 2+1+4+4+2 bytes
    START_MARKER = 0xAA55
    SAMPLE_RATE_HZ = 320  # must match scale.setSampleRate(NAU7802_SPS_320) on the Teensy
    
    # UI
    WINDOW_WIDTH = 800
    WINDOW_HEIGHT = 560
    WINDOW_TITLE = "Motor Control + Data Logger"
    
    # Timing
    COMMAND_COOLDOWN = 0.1  # seconds between commands
    STATUS_DISPLAY_TIME = 10  # seconds to show completion status
    
    # Motors
    INITIAL_SPEED = 2.0  # Float value

    @classmethod
    def init_directories(cls):
        """Create necessary directories"""
        cls.LOG_DIR.mkdir(exist_ok=True)
        return cls.LOG_DIR

# ================= UTILITY FUNCTIONS =================
def crc16(data: bytes) -> int:
    """CRC16-CCITT (XModem) calculation"""
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def check_serial_port(port, baud):
    """Check if serial port is available and not in use"""
    if not os.path.exists(port):
        return False, f"Port {port} does not exist"
    
    try:
        test_ser = serial.Serial(port, baud, timeout=0.1, exclusive=True)
        test_ser.close()
        return True, "Port available"
    except serial.SerialException as e:
        if "could not open port" in str(e):
            return False, f"Port {port} is busy or permission denied"
        return False, str(e)
    except Exception as e:
        return False, str(e)

def kill_modem_manager():
    """Try to stop modem manager which often interferes with serial ports"""
    try:
        subprocess.run(['sudo', 'systemctl', 'stop', 'ModemManager'], 
                      capture_output=True, timeout=5)
        print("✓ Attempted to stop ModemManager")
    except:
        pass

def format_file_size(bytes_size):
    """Format file size in human-readable form"""
    if bytes_size < 1024:
        return f"{bytes_size}B"
    elif bytes_size < 1024 * 1024:
        return f"{bytes_size/1024:.1f}KB"
    else:
        return f"{bytes_size/(1024*1024):.1f}MB"

# ================= DEVICE DISCOVERY MODULE =================
# Both sketches respond to 'I' with a unique ID string (see LONGRUNER_FLASH.ino
# and TEENSY_4_1_FLASH_CRC.ino). This lets us find each device by asking it who
# it is, instead of hardcoding a /dev/ttyACM# that can change on every reboot.
DEVICE_IDS = {
    b"ID:TEENSY_DAQ": "teensy",
    b"ID:LONGRUNER_MOTOR": "motor",
}

def list_candidate_ports():
    """Return USB serial ports worth probing (skips non-USB ports like the Pi's
    onboard GPIO UART, which has no vid/pid and would just hang on a probe)."""
    return [p for p in serial.tools.list_ports.comports() if p.vid is not None]

def identify_port(device_path, config):
    """Open a port, ask 'who are you', and return (role, note).
    role is 'teensy'/'motor'/None. note explains a None result."""
    try:
        with serial.Serial(device_path, config.BAUD, timeout=0.2) as ser:
            time.sleep(config.BOOT_WAIT)  # let boards that reset-on-open finish booting
            ser.reset_input_buffer()
            ser.write(config.IDENTIFY_CMD)
            ser.flush()

            buf = b""
            deadline = time.time() + config.IDENTIFY_TIMEOUT
            while time.time() < deadline:
                if ser.in_waiting:
                    buf += ser.read(ser.in_waiting)
                    for id_bytes, role in DEVICE_IDS.items():
                        if id_bytes in buf:
                            return role, None
                time.sleep(0.05)
            return None, "opened fine but no reply - old firmware without 'I' support, or board is stuck"
    except (serial.SerialException, OSError) as e:
        return None, f"could not open port ({e})"

def discover_devices(config):
    """Probe every candidate USB serial port and return {'teensy': path, 'motor': path}
    for whichever roles were found."""
    candidates = list_candidate_ports()
    print(f"Scanning {len(candidates)} USB serial port(s)...")

    found = {}
    for port in candidates:
        role, note = identify_port(port.device, config)
        if role:
            print(f"  ✓ {port.device} -> {role} ({port.description})")
            found[role] = port.device
        else:
            print(f"  ? {port.device} -> {note} ({port.description})")

    return found, candidates

# ================= DATA LOGGER MODULE =================
class DataLogger:
    """Handles all data logging functionality from Teensy"""
    
    # State constants
    STATE_IDLE = "idle"
    STATE_WAITING_START = "waiting_for_start"
    STATE_WAITING_DONE = "waiting_for_done"
    STATE_WAITING_BIN = "waiting_for_bin_begin"
    STATE_RECEIVING = "receiving_bin"
    STATE_CONVERTING = "converting"
    STATE_COMPLETE = "complete"
    STATE_ERROR = "error"
    
    def __init__(self, port, config):
        self.port = port
        self.config = config
        self.ser = None
        self.reset_state()
        
    def reset_state(self):
        """Reset all state variables"""
        self.logging = False
        self.receiving = False
        self.completed = False
        self.completion_message = ""
        self.valid_packets = 0
        self.corrupted_packets = 0
        self.state = self.STATE_IDLE
        self.binary_mode = False
        self.buffer = b""
        self.silence_output = False
        self.error_count = 0
        self.max_errors = 10
        self.last_successful_read = time.time()
        
    def connect(self):
        """Establish connection to data Teensy"""
        print(f"Connecting to data MCU on {self.port}")
        
        available, message = check_serial_port(self.port, self.config.BAUD)
        if not available:
            print(f"✗ Port check failed: {message}")
            return False
        
        kill_modem_manager()
        
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.config.BAUD
            self.ser.bytesize = serial.EIGHTBITS
            self.ser.parity = serial.PARITY_NONE
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.timeout = 0.1
            self.ser.write_timeout = 0.1
            self.ser.exclusive = False
            self.ser.open()
            print(f"  ✓ Serial port opened")
            
        except serial.SerialException as e:
            print(f"✗ Serial open failed: {e}")
            return False
        except Exception as e:
            print(f"✗ Unexpected error: {e}")
            return False
        
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print("✓ Data MCU connected and ready")
        return True
        
    def start_logging(self):
        """Initiate a 5-second logging session"""
        if self.state != self.STATE_IDLE:
            print(f"Cannot start logging - current state: {self.state}")
            return False
            
        self.state = self.STATE_WAITING_START
        self.completed = False
        self.completion_message = ""
        self.valid_packets = 0
        self.corrupted_packets = 0
        self.binary_mode = False
        self.buffer = b""
        self.silence_output = False
        self.error_count = 0
        self.last_successful_read = time.time()
        
        # Create log files with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bin_file = self.config.LOG_DIR / f"log_{timestamp}.bin"
        self.txt_file = self.config.LOG_DIR / f"log_{timestamp}.txt"
        self.hex_file = self.config.LOG_DIR / f"log_{timestamp}_hex.txt"
        
        print(f"\nStarting data logging for {self.config.LOG_DURATION} seconds...")
        
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b'p')
            self.ser.flush()
            print("  ✓ Start command sent")
        except Exception as e:
            print(f"✗ Failed to send start command: {e}")
            self.state = self.STATE_ERROR
            self.completion_message = f"Failed to send command: {e}"
            self.completed = True
            return False
            
        self.log_start_time = time.time()
        return True
        
    def update(self):
        """Called from main loop to process serial data"""
        if self.state == self.STATE_IDLE:
            return

        current_time = time.time()

        if self.error_count > self.max_errors:
            self._handle_error("Too many communication errors")
            return

        try:
            self._process_incoming_data(current_time)
            self._check_timeouts(current_time)

        except Exception as e:
            self.error_count += 1
            if self.error_count % 5 == 0:
                print(f"⚠ Update error ({self.error_count}/{self.max_errors}): {e}")
    
    def _process_incoming_data(self, current_time):
        """Process any incoming serial data"""
        if not self.ser or not self.ser.is_open:
            self._handle_error("Serial port closed")
            return
            
        if self.ser.in_waiting:
            try:
                data = self.ser.read(self.ser.in_waiting)
                if data:
                    self.buffer += data
                    self.error_count = 0
                    self.last_successful_read = current_time
                    
                    if self.binary_mode:
                        self._process_binary_buffer()
                    else:
                        self._process_text_buffer()
            except serial.SerialException as e:
                self.error_count += 1
                if self.error_count % 5 == 0:
                    print(f"⚠ Serial error ({self.error_count}/{self.max_errors}): {e}")
                time.sleep(0.05)
    
    def _process_text_buffer(self):
        """Process text lines from buffer"""
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            try:
                line_str = line.decode('utf-8', errors='ignore').strip()
                if line_str:
                    if not self.silence_output:
                        print(f"Teensy: {line_str}")
                    self._handle_teensy_message(line_str)
            except:
                pass
                
    def _process_binary_buffer(self):
        """Process binary data buffer"""
        if self.state != self.STATE_RECEIVING:
            return
            
        if b"BIN_END" in self.buffer:
            self._finalize_binary_transfer()
        else:
            # Write chunks to file to avoid memory overflow
            if len(self.buffer) > 1024:
                write_len = len(self.buffer) - 64
                if write_len > 0:
                    data_to_write = self.buffer[:write_len]
                    self.bin_file_handle.write(data_to_write)
                    self.bytes_received += len(data_to_write)
                    self.buffer = self.buffer[write_len:]
                    
            self.last_bin_data_time = time.time()
    
    def _finalize_binary_transfer(self):
        """Complete binary transfer when BIN_END received"""
        parts = self.buffer.split(b"BIN_END", 1)
        binary_data = parts[0]
        remaining = parts[1] if len(parts) > 1 else b""
        
        if binary_data:
            self.bin_file_handle.write(binary_data)
            self.bytes_received += len(binary_data)
            
        self.buffer = remaining
        
        print(f"\n✓ Binary transfer complete: {self.bytes_received} bytes")
        
        self.bin_file_handle.close()
        self.hex_file_handle.close()
        
        self.binary_mode = False
        self.silence_output = False
        
        self.state = self.STATE_CONVERTING
        self._convert_bin_to_txt()
        
        self.state = self.STATE_COMPLETE
        self.completed = True
        self.completion_message = f"Log complete! {self.valid_packets} valid packets"
    
    def _handle_teensy_message(self, line):
        """Handle messages from Teensy based on current state"""
        if self.state == self.STATE_WAITING_START:
            if line == "LOG_START":
                print("✓ Teensy started logging")
                self.state = self.STATE_WAITING_DONE
            elif line == "BIN_BEGIN":
                print("⚠ Received BIN_BEGIN early")
                self._switch_to_binary_mode()
                
        elif self.state == self.STATE_WAITING_DONE:
            if line == "LOG_DONE":
                print("✓ Teensy finished logging")
                self.state = self.STATE_WAITING_BIN
                self.waiting_start_time = time.time()
            elif line == "BIN_BEGIN":
                print("⚠ Received BIN_BEGIN early")
                self._switch_to_binary_mode()
                
        elif self.state == self.STATE_WAITING_BIN:
            if line == "BIN_BEGIN":
                self._switch_to_binary_mode()
            elif line.startswith("ERROR:"):
                print(f"✗ Teensy error: {line}")
                self.state = self.STATE_ERROR
                self.completion_message = f"Teensy error: {line}"
                self.completed = True
                
    def _switch_to_binary_mode(self):
        """Switch to binary reception mode"""
        print("✓ Binary transfer starting...")
        self.silence_output = True
        self.binary_mode = True
        self.state = self.STATE_RECEIVING
        self.bin_file_handle = open(self.bin_file, "wb")
        self.hex_file_handle = open(self.hex_file, "w")
        self.hex_file_handle.write(f"# Binary dump from {datetime.now()}\n")
        self.hex_file_handle.write("#" + "-"*80 + "\n")
        self.bytes_received = 0
        self.last_bin_data_time = time.time()
        self.buffer = b""
    
    def _check_timeouts(self, current_time):
        """Check for various timeout conditions"""
        if self.state == self.STATE_WAITING_START and current_time - self.log_start_time > 3:
            print("⚠ Timeout waiting for LOG_START, continuing...")
            self.state = self.STATE_WAITING_DONE
                
        elif self.state == self.STATE_WAITING_DONE and current_time - self.log_start_time > self.config.LOG_DURATION + 3:
            print("⚠ Timeout waiting for LOG_DONE, assuming complete")
            self.state = self.STATE_WAITING_BIN
            self.waiting_start_time = current_time
                
        elif self.state == self.STATE_WAITING_BIN and hasattr(self, 'waiting_start_time'):
            if current_time - self.waiting_start_time > 3:
                if len(self.buffer) > 0:
                    print(f"  Buffer has {len(self.buffer)} bytes - assuming transfer started")
                    self._switch_to_binary_mode()
                else:
                    self._handle_error("Timeout - Teensy didn't start binary transfer")
                
        elif self.state == self.STATE_RECEIVING and hasattr(self, 'last_bin_data_time'):
            if current_time - self.last_bin_data_time > 3:
                print(f"\n⚠ Timeout during binary transfer")
                print(f"  Received {self.bytes_received} bytes")
                self._finish_binary_transfer()
    
    def _handle_error(self, message):
        """Handle error state"""
        print(f"\n✗ {message}")
        self.state = self.STATE_ERROR
        self.completion_message = message
        self.completed = True
    
    def _finish_binary_transfer(self):
        """Force finish binary transfer on timeout"""
        if self.state == self.STATE_RECEIVING and self.bytes_received > 0:
            print("\n⚠ Forcing transfer completion due to timeout")
            
            if len(self.buffer) > 0:
                self.bin_file_handle.write(self.buffer)
                self.bytes_received += len(self.buffer)
                
            self.bin_file_handle.close()
            self.hex_file_handle.close()
            self.binary_mode = False
            self.silence_output = False
            
            print(f"  Total received: {self.bytes_received} bytes")
            
            self.state = self.STATE_CONVERTING
            self._convert_bin_to_txt()
            
            self.state = self.STATE_COMPLETE
            self.completed = True
            self.completion_message = f"Log complete! {self.valid_packets} valid packets (forced)"
        else:
            self._handle_error("No data received")
    
    def _convert_bin_to_txt(self):
        """Convert binary file to CSV with CRC16 verification"""
        if not self.bin_file.exists():
            print(f"✗ ERROR: binary file not found: {self.bin_file}")
            return
            
        size = self.bin_file.stat().st_size
        print(f"\nConverting {format_file_size(size)} binary file...")
        
        if size == 0:
            print("✗ ERROR: file is empty")
            return
        
        valid_packets = []
        corrupted = 0
        crc_errors = 0
        marker_errors = 0
        
        data = self.bin_file.read_bytes()
        i = 0
        
        while i + self.config.PACKET_SIZE <= len(data):
            chunk = data[i:i+self.config.PACKET_SIZE]
            
            # Unpack
            start_marker, pkt_id, time_ms, reading, crc_recv = struct.unpack('<HBIiH', chunk)
            
            # Validate start marker
            if start_marker != self.config.START_MARKER:
                marker_errors += 1
                i += 1
                continue
            
            # Validate CRC
            crc_calc = crc16(chunk[2:-2])
            if crc_calc != crc_recv:
                crc_errors += 1
                corrupted += 1
                i += self.config.PACKET_SIZE
                continue
            
            # Valid packet
            valid_packets.append((pkt_id, time_ms, reading))
            i += self.config.PACKET_SIZE
        
        self.valid_packets = len(valid_packets)
        self.corrupted_packets = corrupted
        
        if valid_packets:
            self._save_and_report_stats(valid_packets, crc_errors, marker_errors)
        else:
            print(f"\n✗ No valid packets found")
    
    def _save_and_report_stats(self, valid_packets, crc_errors, marker_errors):
        """Save valid packets and generate statistics"""
        with open(self.txt_file, 'w') as f:
            f.write("pkt_id,time_ms,reading\n")
            for pkt_id, t, r in valid_packets:
                f.write(f"{pkt_id},{t},{r}\n")
        
        # Calculate statistics
        times = [t for _, t, _ in valid_packets]
        readings = [r for _, _, r in valid_packets]
        
        if len(times) > 1:
            duration_sec = (times[-1] - times[0]) / 1000.0
            expected_packets = int(self.config.SAMPLE_RATE_HZ * duration_sec)
            packet_loss = max(0, expected_packets - len(valid_packets))
            
            print(f"\n✓ Found {self.valid_packets} valid packets")
            print(f"  - CRC errors: {crc_errors}")
            print(f"  - Marker errors: {marker_errors}")
            print(f"  - Data completeness: {(len(valid_packets)/expected_packets*100):.1f}%")

# ================= MOTOR CONTROLLER MODULE =================
class MotorController:
    """Handles all motor control functionality with throttled speed updates"""
    
    # Key mappings: pygame key constant -> single-char firmware command
    # Motor 1 (X) and Motor 2 (Y) are only reachable via the CoreXY pad now
    # (no individual raw-jog keys for them - see COREXY_DIRECTIONS below).
    # Stage (Motor 3 / Z) = W/S | Injection (Motor 4 / A) = R/F
    # (the firmware command letters themselves are unchanged - only which physical
    # key sends them changed, so LONGRUNER_FLASH_v0.ino didn't need updating here)
    KEY_TO_CMD = {
        pygame.K_w: 'e',  pygame.K_s: 'd',  # Stage (Z): W fwd / S rev
        pygame.K_r: 'r',  pygame.K_f: 'f',  # Injection (A): R fwd / F rev
    }

    # Stage + Injection commands - also used for their touchscreen up/down buttons
    MOTOR_CMDS = ['e', 'd', 'r', 'f']

    # ---- CoreXY (Motor 1 = belt 1 / X slot, Motor 2 = belt 2 / Y slot) ----
    # Motor 1 and Motor 2 aren't independent orthogonal axes - they're a CoreXY
    # belt pair, so jogging "Motor 1" alone actually moves the carriage diagonally.
    # Standard CoreXY kinematics (see https://corexy.com/theory.html, and the same
    # math used by Marlin/Klipper's CoreXY kinematics modules) is beltA=dx+dy,
    # beltB=dx-dy. For jogging we only care about the SIGN of each belt's delta.
    #
    # NOTE: on this build, real-world East/West (and therefore the diagonals)
    # came out mirrored versus the textbook dx convention - N/S tested correct,
    # E/W tested backwards. Rather than fight the belt wiring, the table below
    # has E/W and the NE<->NW / SE<->SW pairs swapped from the textbook values
    # so the on-screen compass direction matches the actual carriage motion.
    COREXY_DIRECTIONS = {
        'N':  (1, -1),  'S':  (-1, 1),
        'E':  (-1, -1), 'W':  (1, 1),
        'NE': (0, -1),  'SW': (0, 1),
        'NW': (1, 0),   'SE': (-1, 0),
    }
    # Belt sign -> firmware command. Motor 1 = belt 1, Motor 2 = belt 2.
    # 'Q'/'W' are the firmware's "stop just this one motor" commands - needed
    # for diagonals, where one belt must hold still while the other turns.
    BELT1_CMD = {1: 'q', -1: 'a', 0: 'Q'}
    BELT2_CMD = {1: 'w', -1: 's', 0: 'W'}

    def __init__(self, port, config):
        self.port = port
        self.config = config
        self.ser = None
        self.current_speed = 2
        self.current_mode = 'c'

        # Track pressed states - keyboard
        self.pressed_keys = {key: False for key in self.KEY_TO_CMD}

        # Track pressed states - touchscreen buttons (keyed by the same command letters)
        self.pressed_buttons = {cmd: False for cmd in self.MOTOR_CMDS}

        # Currently-held CoreXY compass direction (None if the pad isn't in use)
        self.active_corexy = None

        # Speed throttling
        self.last_speed_send = 0
        self.speed_throttle_ms = 150  # Only send speed every 150ms
        self.pending_speed = None

        # Continuous movement tracking
        self.last_movement_send = 0
        self.movement_interval = 0.1  # 100ms between movement commands
        
    def connect(self):
        """Establish connection to motor controller"""
        print(f"Connecting to motor MCU on {self.port}")
        try:
            self.ser = serial.Serial(self.port, self.config.BAUD, timeout=0.1, write_timeout=0.1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print("✓ Motor MCU connected")
            return True
        except Exception as e:
            print(f"✗ Motor connection failed: {e}")
            return False
            
    def send_command(self, cmd):
        """Send a single-character command to the motor controller.
        The firmware reads one byte per loop, so every command here must be
        exactly one meaningful character (movement letters, ' ', 'm', '1'/'2'/'3')."""
        try:
            if isinstance(cmd, int):
                cmd = str(cmd)

            if not cmd.endswith('\n'):
                cmd = cmd + '\n'

            self.ser.write(cmd.encode('ascii'))
            self.ser.flush()
        except Exception as e:
            print(f"Failed to send command '{cmd}': {e}")

    def set_speed(self, speed):
        """Set motor speed. The firmware only supports 3 discrete speeds
        (see stepDelays[] in LONGRUNER_FLASH.ino), so we always round and
        clamp to a whole number in {1, 2, 3} before sending."""
        speed = int(round(speed))
        speed = max(1, min(3, speed))
        if speed != self.current_speed:
            self.current_speed = speed
            self.pending_speed = speed
    
    def update(self):
        """Call this regularly to send throttled commands"""
        current_time = time.time()
        
        # Send throttled speed updates
        if self.pending_speed is not None:
            if current_time - self.last_speed_send > (self.speed_throttle_ms / 1000.0):
                self.send_command(str(self.pending_speed))
                self.last_speed_send = current_time
                self.pending_speed = None
        
        # Send continuous movement commands (throttled)
        if self.current_mode == 'c':
            if current_time - self.last_movement_send > self.movement_interval:
                # Check keyboard presses
                for key, pressed in self.pressed_keys.items():
                    if pressed:
                        self.send_command(self.KEY_TO_CMD[key])
                        self.last_movement_send = current_time
                        return
                
                # Check touchscreen button presses
                for cmd, pressed in self.pressed_buttons.items():
                    if pressed:
                        self.send_command(cmd)
                        self.last_movement_send = current_time
                        return

                # Check CoreXY compass pad
                if self.active_corexy:
                    self._send_corexy(self.active_corexy)
                    self.last_movement_send = current_time

    def _send_corexy(self, direction):
        """Send both belt commands for a CoreXY compass direction."""
        belt1_sign, belt2_sign = self.COREXY_DIRECTIONS[direction]
        self.send_command(self.BELT1_CMD[belt1_sign])
        self.send_command(self.BELT2_CMD[belt2_sign])

    def corexy_pressed(self, direction):
        """Start (or refresh) a coordinated X/Y move in one of 8 compass
        directions. Held down + resent periodically + explicit per-belt stop
        on release, same pattern as the individual motor jog controls."""
        if direction not in self.COREXY_DIRECTIONS:
            return
        self.active_corexy = direction
        self._send_corexy(direction)
        self.last_movement_send = time.time()

    def corexy_released(self, direction):
        """Stop the CoreXY pad - only if this was the direction actually
        driving it (guards against a stray release event from a different
        button after a quick direction change)."""
        if self.active_corexy == direction:
            self.send_command('Q')  # stop Motor 1 (belt 1)
            self.send_command('W')  # stop Motor 2 (belt 2)
            self.active_corexy = None

    def key_pressed(self, key):
        """Handle key press"""
        if key in self.pressed_keys:
            self.pressed_keys[key] = True
            # Send immediately for responsiveness
            self.send_command(self.KEY_TO_CMD[key])
            self.last_movement_send = time.time()
    
    def key_released(self, key):
        """Handle key release for continuous movement"""
        if key in self.pressed_keys:
            self.pressed_keys[key] = False
            # Check if ANY input is still pressed (keys or buttons)
            any_keys_pressed = any(self.pressed_keys.values())
            any_buttons_pressed = any(self.pressed_buttons.values())
            
            if self.current_mode == 'c' and not any_keys_pressed and not any_buttons_pressed:
                self.stop()
    
    def button_pressed(self, cmd):
        """Handle touchscreen button press. cmd is the single-char firmware
        command for that motor/direction (e.g. 'q' for Motor 1 forward)."""
        if cmd in self.pressed_buttons:
            self.pressed_buttons[cmd] = True
            self.send_command(cmd)
            self.last_movement_send = time.time()

    def button_released(self, cmd):
        """Handle touchscreen button release for continuous movement"""
        if cmd in self.pressed_buttons:
            self.pressed_buttons[cmd] = False
            # Check if ANY input is still pressed (keys or buttons)
            any_keys_pressed = any(self.pressed_keys.values())
            any_buttons_pressed = any(self.pressed_buttons.values())
            
            if self.current_mode == 'c' and not any_keys_pressed and not any_buttons_pressed:
                self.stop()
    
    def toggle_mode(self):
        """Toggle between continuous and fixed mode"""
        self.send_command('m')
        self.current_mode = 'c' if self.current_mode == 'f' else 'f'
        return self.current_mode
    
    def stop(self):
        """Stop all motors"""
        self.send_command(' ')
        self.active_corexy = None
        for key in self.pressed_keys:
            self.pressed_keys[key] = False
        for btn in self.pressed_buttons:
            self.pressed_buttons[btn] = False
    
    def read_responses(self):
        """Read and print any responses. Previously this silently dropped
        CONT:/FIXED:/SPEED: acknowledgements, which made it impossible to tell
        from the console whether a given motor command actually reached the
        firmware. Printing everything makes that visible for troubleshooting."""
        try:
            if self.ser and self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    print(f"Motor: {line}")
        except:
            pass

# ================= UI MODULE =================
class Slider:
    """Custom slider with 0.1 increments for stability"""
    
    def __init__(self, x, y, width, height, min_val=1.0, max_val=3.0, initial=2.0):
        self.rect = pygame.Rect(x, y, width, height)
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial
        # Firmware only supports 3 discrete speeds (see stepDelays[] in
        # LONGRUNER_FLASH.ino), so the slider snaps to whole numbers only.
        self.step = 1.0
        
        # Handle size
        self.handle_radius = height + 4
        self.handle_x = self._value_to_x(initial)
        self.handle_y = y + height // 2
        
        self.dragging = False
        self.hovered = False
        
        # Colors
        self.track_color = (60, 60, 70)
        self.fill_color = (100, 200, 255)
        self.handle_color = (220, 220, 240)
        self.handle_hover_color = (255, 255, 255)
    
    def _value_to_x(self, value):
        t = (value - self.min_val) / (self.max_val - self.min_val)
        return self.rect.x + int(t * self.rect.width)
    
    def _x_to_value(self, x):
        t = max(0, min(1, (x - self.rect.x) / self.rect.width))
        # Round to nearest 0.1
        raw_value = self.min_val + t * (self.max_val - self.min_val)
        stepped_value = round(raw_value / self.step) * self.step
        return max(self.min_val, min(self.max_val, stepped_value))
    
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                handle_rect = pygame.Rect(self.handle_x - self.handle_radius, 
                                         self.handle_y - self.handle_radius,
                                         self.handle_radius * 2, self.handle_radius * 2)
                if handle_rect.collidepoint(event.pos):
                    self.dragging = True
                    return True
                elif self.rect.collidepoint(event.pos):
                    self.handle_x = max(self.rect.x, min(self.rect.x + self.rect.width, event.pos[0]))
                    self.value = self._x_to_value(self.handle_x)
                    self.dragging = True
                    return True
                    
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False
                
        elif event.type == pygame.MOUSEMOTION:
            handle_rect = pygame.Rect(self.handle_x - self.handle_radius, 
                                     self.handle_y - self.handle_radius,
                                     self.handle_radius * 2, self.handle_radius * 2)
            self.hovered = handle_rect.collidepoint(event.pos) or self.rect.collidepoint(event.pos)
            
            if self.dragging:
                self.handle_x = max(self.rect.x, min(self.rect.x + self.rect.width, event.pos[0]))
                self.value = self._x_to_value(self.handle_x)
                return True
        
        return False
    
    def draw(self, screen):
        # Draw track background
        pygame.draw.rect(screen, self.track_color, self.rect, border_radius=8)
        
        # Draw filled portion
        fill_width = self.handle_x - self.rect.x
        if fill_width > 0:
            fill_rect = pygame.Rect(self.rect.x, self.rect.y, fill_width, self.rect.height)
            pygame.draw.rect(screen, self.fill_color, fill_rect, border_radius=8)
        
        # Draw handle
        handle_color = self.handle_hover_color if self.hovered or self.dragging else self.handle_color
        pygame.draw.circle(screen, handle_color, (self.handle_x, self.handle_y), self.handle_radius)
        pygame.draw.circle(screen, (255, 255, 255), (self.handle_x, self.handle_y), self.handle_radius, 1)
        
        # Draw speed value as a whole number (matches the 3 discrete firmware speeds)
        font = pygame.font.Font(None, 18)
        value_text = font.render(f"{int(self.value)}", True, (255, 255, 255))
        text_rect = value_text.get_rect(center=(self.handle_x, self.handle_y))
        screen.blit(value_text, text_rect)
        
        # Draw min and max labels
        font_small = pygame.font.Font(None, 14)
        min_text = font_small.render(f"{int(self.min_val)}", True, (120,120,130))
        max_text = font_small.render(f"{int(self.max_val)}", True, (120,120,130))
        screen.blit(min_text, (self.rect.x - 20, self.rect.y + 2))
        screen.blit(max_text, (self.rect.x + self.rect.width + 8, self.rect.y + 2))
    
    def get_value(self):
        return self.value

class ToggleSwitch:
    """Toggle switch for continuous/fixed mode"""
    
    def __init__(self, x, y, width=60, height=30, initial_state='c'):
        self.rect = pygame.Rect(x, y, width, height)
        self.state = initial_state  # 'c' for continuous, 'f' for fixed
        self.hovered = False
        self.anim_offset = 0
        
        # Colors
        self.bg_color_continuous = (70, 140, 70)  # Green for continuous
        self.bg_color_fixed = (140, 100, 70)      # Brown/amber for fixed
        self.handle_color = (240, 240, 240)
        self.handle_hover_color = (255, 255, 255)
        
    def toggle(self):
        """Toggle the switch state"""
        self.state = 'f' if self.state == 'c' else 'c'
        return self.state
    
    def set_state(self, state):
        """Set switch state directly"""
        if state in ['c', 'f']:
            self.state = state
    
    def handle_event(self, event):
        """Handle mouse events"""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.rect.collidepoint(event.pos):
                return self.toggle()
        return None
    
    def draw(self, screen):
        """Draw the toggle switch"""
        # Determine background color based on state
        bg_color = self.bg_color_continuous if self.state == 'c' else self.bg_color_fixed
        
        # Draw background with rounded corners
        pygame.draw.rect(screen, bg_color, self.rect, border_radius=15)
        pygame.draw.rect(screen, (100,100,100), self.rect, 2, border_radius=15)
        
        # Draw handle (position depends on state)
        handle_x = self.rect.x + (self.rect.width - self.rect.height) if self.state == 'c' else self.rect.x
        handle_rect = pygame.Rect(handle_x, self.rect.y, self.rect.height, self.rect.height)
        
        # Handle color
        handle_color = self.handle_hover_color if self.hovered else self.handle_color
        
        # Draw handle with slight rounding
        pygame.draw.rect(screen, handle_color, handle_rect, border_radius=12)
        pygame.draw.rect(screen, (150,150,150), handle_rect, 1, border_radius=12)
        
        # Draw mode indicator text
        font = pygame.font.Font(None, 18)
        cont_text = font.render("CONT", True, (255,255,255) if self.state == 'c' else (150,150,150))
        fixed_text = font.render("FIXED", True, (255,255,255) if self.state == 'f' else (150,150,150))
        
        # Position text
        screen.blit(cont_text, (self.rect.x + 8, self.rect.y + 8))
        screen.blit(fixed_text, (self.rect.x + self.rect.width - 45, self.rect.y + 8))


class CountdownButton:
    """Button with circular countdown animation"""
    
    def __init__(self, x, y, radius, color, hover_color, text="LOG", font_size=28):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.hover_color = hover_color
        self.text = text
        self.font = pygame.font.Font(None, font_size)
        
        self.hovered = False
        self.pressed = False
        
        # Countdown state
        self.countdown_active = False
        self.countdown_start = 0
        self.countdown_duration = 5  # 5 seconds pre-log delay
        self.countdown_angle = 0
        
        # Logging state
        self.logging_active = False
        self.logging_start = 0
        self.logging_duration = 5  # 5 seconds logging
        
    def start_countdown(self):
        """Start the pre-log countdown"""
        self.countdown_active = True
        self.countdown_start = time.time()
        self.countdown_angle = 0
        
    def update(self):
        """Update countdown and logging states"""
        current_time = time.time()
        
        if self.countdown_active:
            elapsed = current_time - self.countdown_start
            progress = min(1.0, elapsed / self.countdown_duration)
            self.countdown_angle = 360 * progress
            
            if elapsed >= self.countdown_duration:
                self.countdown_active = False
                self.logging_active = True
                self.logging_start = current_time
                
        elif self.logging_active:
            elapsed = current_time - self.logging_start
            if elapsed >= self.logging_duration:
                self.logging_active = False
    
    def is_counting_down(self):
        """Check if button is in countdown state"""
        return self.countdown_active
    
    def is_logging(self):
        """Check if button is in logging state"""
        return self.logging_active
    
    def get_countdown_time(self):
        """Get remaining countdown time"""
        if self.countdown_active:
            elapsed = time.time() - self.countdown_start
            return max(0, self.countdown_duration - elapsed)
        return 0
    
    def get_logging_time(self):
        """Get remaining logging time"""
        if self.logging_active:
            elapsed = time.time() - self.logging_start
            return max(0, self.logging_duration - elapsed)
        return 0
    
    def handle_event(self, event):
        """Handle mouse events"""
        # Check if mouse is over button
        mouse_pos = pygame.mouse.get_pos()
        dx = mouse_pos[0] - self.x
        dy = mouse_pos[1] - self.y
        distance = (dx*dx + dy*dy) ** 0.5
        self.hovered = distance <= self.radius
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.hovered:
                self.pressed = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.pressed = False
            
        return False
    
    def draw(self, screen):
        """Draw the button with countdown ring"""
        # Draw button background
        color = self.hover_color if self.hovered else self.color
        if self.pressed:
            color = tuple(max(0, c - 40) for c in color)
        
        # Draw main button
        pygame.draw.circle(screen, color, (self.x, self.y), self.radius)
        pygame.draw.circle(screen, (255, 255, 255), (self.x, self.y), self.radius, 2)
        
        # Draw text
        text_surf = self.font.render(self.text, True, (255, 255, 255))
        text_rect = text_surf.get_rect(center=(self.x, self.y))
        screen.blit(text_surf, text_rect)
        
        # Draw countdown ring (if active)
        if self.countdown_active:
            # Calculate end angle (clockwise from top)
            start_angle = -90  # Start from top
            end_angle = start_angle + self.countdown_angle
            
            # Draw the countdown arc
            rect = pygame.Rect(self.x - self.radius - 5, self.y - self.radius - 5,
                              (self.radius + 10) * 2, (self.radius + 10) * 2)
            pygame.draw.arc(screen, (255, 255, 0), rect, 
                          start_angle * 3.14159 / 180, 
                          end_angle * 3.14159 / 180, 4)
            
            # Draw countdown text
            font_small = pygame.font.Font(None, 20)
            time_left = self.get_countdown_time()
            time_text = font_small.render(f"{time_left:.1f}s", True, (255, 255, 200))
            time_rect = time_text.get_rect(center=(self.x, self.y + self.radius + 20))
            screen.blit(time_text, time_rect)
        
        # Draw logging timer (if active)
        elif self.logging_active:
            font_small = pygame.font.Font(None, 20)
            time_left = self.get_logging_time()
            time_text = font_small.render(f"{time_left:.1f}s", True, (100, 255, 100))
            time_rect = time_text.get_rect(center=(self.x, self.y + self.radius + 20))
            screen.blit(time_text, time_rect)

class KeyButton:
    """Simple key button with letter - improved release detection"""
    
    def __init__(self, x, y, size, letter, color=(50,50,60), hover_color=(80,80,100)):
        self.rect = pygame.Rect(x, y, size, size)
        self.letter = letter
        self.color = color
        self.hover_color = hover_color
        self.font = pygame.font.Font(None, 32)
        self.hovered = False
        self.pressed = False
        self.pressed_id = None  # Track which mouse button pressed it
        
    def draw(self, screen):
        """Draw the key button"""
        color = self.hover_color if self.hovered else self.color
        if self.pressed:
            color = tuple(max(0, c - 30) for c in color)
        
        # Draw button with rounded corners
        pygame.draw.rect(screen, color, self.rect, border_radius=12)
        pygame.draw.rect(screen, (150,150,150), self.rect, 2, border_radius=12)
        
        # Draw letter
        text_surf = self.font.render(self.letter, True, (220,220,240))
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)
    
    def handle_event(self, event):
        """Handle mouse events - returns (pressed, released) tuple"""
        released = False
        
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
            
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.rect.collidepoint(event.pos):
                self.pressed = True
                self.pressed_id = event.button
                return (True, False)  # (pressed, released)
                
        elif event.type == pygame.MOUSEBUTTONUP:
            # Check if this was the button that pressed it
            if self.pressed and event.button == self.pressed_id:
                self.pressed = False
                self.pressed_id = None
                released = True
            # Also check if mouse is over button on release
            elif self.rect.collidepoint(event.pos) and event.button == 1:
                released = True
        
        return (False, released)

class Button:
    """Simple button for stop and other actions"""
    
    def __init__(self, x, y, width, height, text, color, hover_color, text_color=(255,255,255), font_size=24, border_radius=8):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.text_color = text_color
        self.font = pygame.font.Font(None, font_size)
        self.border_radius = border_radius
        self.hovered = False
        self.pressed = False
        
    def draw(self, screen):
        """Draw the button on screen"""
        color = self.hover_color if self.hovered else self.color
        if self.pressed:
            color = tuple(max(0, c - 40) for c in color)
            
        pygame.draw.rect(screen, color, self.rect, border_radius=self.border_radius)
        pygame.draw.rect(screen, (150,150,150), self.rect, 2, border_radius=self.border_radius)
        
        text_surf = self.font.render(self.text, True, self.text_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)
        
    def handle_event(self, event):
        """Handle mouse events for the button"""
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1 and self.rect.collidepoint(event.pos):
                self.pressed = True
                return True
        elif event.type == pygame.MOUSEBUTTONUP:
            if self.pressed:
                self.pressed = False
                return True
        return False


class UI:
    """Main UI manager - Clean, minimal design"""
    
    def __init__(self, config, motor_controller, data_logger):
        self.config = config
        self.motor = motor_controller
        self.logger = data_logger

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
        pygame.display.set_caption("Motor Control")
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 16)
        self.clock = pygame.time.Clock()
        
        # UI state
        self.last_continuous_send = 0
        self.continuous_interval = 0.1
        
        # Create UI elements
        self._create_ui_elements()
        
    def _create_ui_elements(self):
        """Create all UI elements"""
        center_x = self.config.WINDOW_WIDTH // 2

        # Speed control - small, top-left
        self.speed_slider = Slider(40, 55, 130, 14, 1.0, 3.0, self.motor.current_speed)

        # Continuous/fixed mode toggle - directly underneath the speed control
        self.mode_toggle = ToggleSwitch(40, 90, 85, 26, self.motor.current_mode)

        # Pressure collection (log) button - small, top-right
        self.log_btn = CountdownButton(self.config.WINDOW_WIDTH - 55, 55, 25, (0,100,120), (0,150,180))

        # CoreXY compass pad (grey) - pushed to the left and slightly smaller so
        # Stage/Injection can sit beside it instead of underneath. Motor 1 + Motor 2
        # are a CoreXY belt pair, so clicking a direction here drives BOTH motors
        # together (per the COREXY_DIRECTIONS transform on MotorController) to move
        # the carriage in that real-world direction, rather than jogging one belt
        # alone. This is the ONLY way to move Motor 1/2 - no individual raw-jog buttons.
        pad_cx, pad_cy = 210, 280
        spacing = 62
        pad_size = 48
        pad_positions = {
            'NW': (pad_cx - spacing, pad_cy - spacing), 'N': (pad_cx, pad_cy - spacing), 'NE': (pad_cx + spacing, pad_cy - spacing),
            'W':  (pad_cx - spacing, pad_cy),                                            'E':  (pad_cx + spacing, pad_cy),
            'SW': (pad_cx - spacing, pad_cy + spacing), 'S': (pad_cx, pad_cy + spacing), 'SE': (pad_cx + spacing, pad_cy + spacing),
        }
        pad_color = (70, 70, 78)
        pad_hover = (100, 100, 110)
        self.corexy_btns = {
            direction: KeyButton(x - pad_size // 2, y - pad_size // 2, pad_size, direction,
                                  color=pad_color, hover_color=pad_hover)
            for direction, (x, y) in pad_positions.items()
        }
        # One-shot stop button at the center of the pad
        stop_size = 40
        self.corexy_stop_btn = Button(pad_cx - stop_size // 2, pad_cy - stop_size // 2,
                                       stop_size, stop_size, "■", (90, 40, 40), (140, 60, 60), font_size=18)
        self.corexy_label_pos = (pad_cx, 178)

        # Stage (Motor 3 / Z) - blue up/down pair
        # Injection (Motor 4 / A) - red up/down pair
        # Positioned to the right of the CoreXY pad, not underneath it
        linear_size = 48
        stage_x = 430
        injection_x = 570
        up_y, down_y = 240, 340
        stage_color, stage_hover = (40, 70, 140), (65, 100, 180)
        injection_color, injection_hover = (140, 40, 40), (180, 65, 65)

        self.linear_btns = {
            'e': KeyButton(stage_x - linear_size // 2, up_y - linear_size // 2, linear_size, '▲',
                            color=stage_color, hover_color=stage_hover),
            'd': KeyButton(stage_x - linear_size // 2, down_y - linear_size // 2, linear_size, '▼',
                            color=stage_color, hover_color=stage_hover),
            'r': KeyButton(injection_x - linear_size // 2, up_y - linear_size // 2, linear_size, '▲',
                            color=injection_color, hover_color=injection_hover),
            'f': KeyButton(injection_x - linear_size // 2, down_y - linear_size // 2, linear_size, '▼',
                            color=injection_color, hover_color=injection_hover),
        }
        # Remember label positions for draw()
        self.stage_label_pos = (stage_x, 195)
        self.injection_label_pos = (injection_x, 195)

    def handle_events(self):
        """Process all UI events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            # Keyboard events
            if event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
                self._handle_keyboard(event)
            
            # Slider events
            if self.speed_slider.handle_event(event):
                new_speed = self.speed_slider.get_value()
                self.motor.set_speed(new_speed)
            
            # Mode toggle events
            result = self.mode_toggle.handle_event(event)
            if result is not None:
                self.motor.current_mode = result
                self.motor.send_command('m')
            
            # Log button events
            if self.log_btn.handle_event(event) and event.type == pygame.MOUSEBUTTONDOWN:
                if self.logger.state == DataLogger.STATE_IDLE:
                    self.log_btn.start_countdown()
            
            # Stage + Injection button events (touchscreen up/down pairs)
            for cmd, btn in self.linear_btns.items():
                pressed, released = btn.handle_event(event)

                if pressed:
                    self.motor.button_pressed(cmd)
                if released:
                    self.motor.button_released(cmd)

            # CoreXY compass pad events
            for direction, btn in self.corexy_btns.items():
                pressed, released = btn.handle_event(event)

                if pressed:
                    self.motor.corexy_pressed(direction)
                if released:
                    self.motor.corexy_released(direction)

            # CoreXY pad stop button (one-shot click, not hold-to-repeat)
            if self.corexy_stop_btn.handle_event(event) and event.type == pygame.MOUSEBUTTONDOWN:
                self.motor.stop()

            # Also handle mouse button up anywhere on screen to catch releases outside buttons
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left mouse button
                    # Check if any button is still pressed and release it
                    for cmd, btn in self.linear_btns.items():
                        if btn.pressed:
                            btn.pressed = False
                            btn.pressed_id = None
                            self.motor.button_released(cmd)
                    for direction, btn in self.corexy_btns.items():
                        if btn.pressed:
                            btn.pressed = False
                            btn.pressed_id = None
                            self.motor.corexy_released(direction)

        return True
    
    def _handle_keyboard(self, event):
        """Handle keyboard input"""
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                return False
            elif event.key == pygame.K_SPACE:
                self.motor.stop()
            elif event.key == pygame.K_m:
                self.motor.toggle_mode()
                self.mode_toggle.set_state(self.motor.current_mode)
            elif event.key in [pygame.K_1, pygame.K_2, pygame.K_3]:
                speed_map = {pygame.K_1: 1, pygame.K_2: 2, pygame.K_3: 3}
                self.motor.set_speed(speed_map[event.key])
            elif event.key == pygame.K_p:
                # Was documented in the printed controls but never actually wired up
                if self.logger.state == DataLogger.STATE_IDLE:
                    self.log_btn.start_countdown()
            elif event.key == pygame.K_SLASH:
                # Diagnostic: ask the firmware what it thinks its current state is
                print("→ Requesting motor status...")
                self.motor.send_command('?')
            elif event.key == pygame.K_t:
                # Diagnostic: pulse every motor's STEP/DIR pins in turn (~8s) so you
                # can confirm with a multimeter/LED that the Arduino is actually
                # driving each pin, independent of the CNC shield's jumpers
                print("→ Running motor pin test (~8s, all motors, watch console)...")
                self.motor.send_command('t')
            elif event.key in self.motor.KEY_TO_CMD:
                self.motor.key_pressed(event.key)
        elif event.type == pygame.KEYUP:
            if event.key in self.motor.KEY_TO_CMD:
                self.motor.key_released(event.key)

        return True
    
    def update(self):
        """Update UI state"""
        current_time = time.time()
        
        # Update motor controller (sends throttled commands)
        self.motor.update()

        # Update log button state
        self.log_btn.update()
        
        # Start actual logging when countdown finishes
        if self.log_btn.is_logging() and self.logger.state == DataLogger.STATE_IDLE:
            self.logger.start_logging()
        
        # Update slider to match motor speed if changed elsewhere
        slider_val = self.speed_slider.get_value()
        motor_val = self.motor.current_speed
        
        if abs(slider_val - motor_val) > 0.05:
            self.speed_slider.value = motor_val
            self.speed_slider.handle_x = self.speed_slider._value_to_x(motor_val)
        
        # Update toggle to match motor mode if changed elsewhere
        if self.mode_toggle.state != self.motor.current_mode:
            self.mode_toggle.set_state(self.motor.current_mode)
    
    def draw(self):
        """Draw the complete UI"""
        self.screen.fill((15, 15, 20))  # Dark background
        
        center_x = self.config.WINDOW_WIDTH // 2

        # Draw speed control (top-left) with subtle label
        slider_label = self.small_font.render("SPEED", True, (120,120,140))
        self.screen.blit(slider_label, (40, 38))
        self.speed_slider.draw(self.screen)

        # Draw mode toggle (underneath the speed control)
        self.mode_toggle.draw(self.screen)

        # Draw pressure collection (log) button (top-right)
        self.log_btn.draw(self.screen)

        # Draw CoreXY compass pad section (grey)
        corexy_label = self.small_font.render("COREXY PAD (X/Y)", True, (100,100,115))
        corexy_rect = corexy_label.get_rect(center=self.corexy_label_pos)
        self.screen.blit(corexy_label, corexy_rect)

        for btn in self.corexy_btns.values():
            btn.draw(self.screen)
        self.corexy_stop_btn.draw(self.screen)

        # Draw Stage (blue) and Injection (red) sections
        stage_label = self.small_font.render("STAGE (Z)", True, (120,150,220))
        stage_rect = stage_label.get_rect(center=self.stage_label_pos)
        self.screen.blit(stage_label, stage_rect)

        injection_label = self.small_font.render("INJECTION (A)", True, (220,140,140))
        injection_rect = injection_label.get_rect(center=self.injection_label_pos)
        self.screen.blit(injection_label, injection_rect)

        for btn in self.linear_btns.values():
            btn.draw(self.screen)

        # Draw minimal connection indicators
        motor_status = "●" if self.motor.ser and self.motor.ser.is_open else "○"
        data_status = "●" if self.logger.ser and self.logger.ser.is_open else "○"
        
        motor_dot = self.small_font.render(motor_status, True, (100,255,100) if self.motor.ser else (100,100,100))
        data_dot = self.small_font.render(data_status, True, (100,255,100) if self.logger.ser else (100,100,100))
        
        self.screen.blit(motor_dot, (20, 20))
        self.screen.blit(data_dot, (40, 20))
        
        # Draw logger status as minimal icon
        if self.logger.state == DataLogger.STATE_RECEIVING:
            status_text = self.small_font.render("⬇", True, (255,220,100))
            self.screen.blit(status_text, (60, 20))
        elif hasattr(self.logger, 'valid_packets') and self.logger.valid_packets > 0:
            status_text = self.small_font.render("✓", True, (100,255,100))
            self.screen.blit(status_text, (60, 20))
        
        pygame.display.flip()
    
    def run(self):
        """Main UI loop"""
        running = True
        while running:
            # Handle events
            running = self.handle_events()
            
            # Update logger
            self.logger.update()
            
            # Update UI state
            self.update()
            
            # Read motor responses
            self.motor.read_responses()
            
            # Draw everything - THIS CALLS THE DRAW METHOD
            self.draw()
            
            # Cap framerate
            self.clock.tick(30)
        
        return True
    
    def cleanup(self):
        """Clean up UI resources"""
        pygame.quit()
# ================= MAIN APPLICATION =================
class Application:
    """Main application orchestrator"""
    
    def __init__(self):
        self.config = Config()
        self.motor = None
        self.logger = None
        self.ui = None
        
    def initialize(self, motor_port_override=None, data_port_override=None):
        """Initialize all components"""
        print("\n" + "="*60)
        print("TEENSY MOTOR + DATA LOGGER (CRC16)")
        print("="*60)
        print(f"Log directory: {self.config.LOG_DIR}")
        print("="*60 + "\n")

        # Create log directory
        self.config.init_directories()

        # Find the devices (or use manual overrides)
        if not self._resolve_ports(motor_port_override, data_port_override):
            return False

        # Initialize hardware
        if not self._initialize_hardware():
            return False

        # Initialize UI
        self.ui = UI(self.config, self.motor, self.logger)

        return True

    def _resolve_ports(self, motor_port_override, data_port_override):
        """Figure out which port is which device, either via override flags
        or by auto-discovery (asking each board to identify itself)."""
        if motor_port_override and data_port_override:
            print(f"Using manual ports: motor={motor_port_override}, data={data_port_override}")
            self.config.MOTOR_PORT = motor_port_override
            self.config.DATA_PORT = data_port_override
            return True

        found, candidates = discover_devices(self.config)

        self.config.MOTOR_PORT = motor_port_override or found.get('motor')
        self.config.DATA_PORT = data_port_override or found.get('teensy')

        if self.config.MOTOR_PORT and self.config.DATA_PORT:
            print(f"\nMotor port: {self.config.MOTOR_PORT}")
            print(f"Data port:  {self.config.DATA_PORT}")
            return True

        print("\n✗ Could not identify both devices.")
        if not self.config.DATA_PORT:
            print("  Missing: Teensy DAQ")
        if not self.config.MOTOR_PORT:
            print("  Missing: Longruner motor controller")
        if candidates:
            print("\nPorts found but not confirmed:")
            for p in candidates:
                print(f"  {p.device} - {p.description}")
        else:
            print("\nNo USB serial ports found at all.")
        print("\n💡 Checklist:")
        print("   - Are both boards plugged in and powered?")
        print("   - Permissions: sudo usermod -a -G dialout $USER (logout required)")
        print("   - Or pin ports manually: --motor-port /dev/ttyACM0 --data-port /dev/ttyACM1")
        return False

    def _initialize_hardware(self):
        """Initialize motor controller and data logger"""
        self.motor = MotorController(self.config.MOTOR_PORT, self.config)
        self.logger = DataLogger(self.config.DATA_PORT, self.config)

        motor_connected = self.motor.connect()
        data_connected = self.logger.connect()

        if not motor_connected or not data_connected:
            print("\n✗ Failed to connect to devices")
            return False

        return True
    
    def run(self):
        """Run the main application"""
        print("\nControls:")
        print("  Click buttons or use keyboard:")
        print("  CoreXY pad (click a direction) = Motor 1+2 (X/Y) coordinated motion")
        print("  W/S = Stage (Z) | R/F = Injection (A)")
        print("  1/2/3 = speed | M = mode | SPACE = stop | P = log | ESC = quit")
        print("  / = motor status query | T = pin test (probe each motor's wiring)")
        print("="*60 + "\n")
        
        # Run UI
        result = self.ui.run()
        
        return result
    
    def cleanup(self):
        """Clean up all resources"""
        print("\n" + "="*60)
        print("SHUTTING DOWN")
        print("="*60)
        
        print("Stopping motors...")
        if self.motor:
            self.motor.stop()
            time.sleep(0.1)

            if self.motor.ser and self.motor.ser.is_open:
                self.motor.ser.close()
                print("✓ Motor serial closed")

        if self.logger and self.logger.ser and self.logger.ser.is_open:
            self.logger.ser.close()
            print("✓ Logger serial closed")
        
        if self.ui:
            self.ui.cleanup()
        
        print("\n" + "="*60)
        print("Program terminated")

# ================= ENTRY POINT =================
def parse_args():
    parser = argparse.ArgumentParser(description="Teensy motor control + data logger")
    parser.add_argument('--motor-port', help="Manually pin the motor controller port (skips auto-discovery for it)")
    parser.add_argument('--data-port', help="Manually pin the Teensy DAQ port (skips auto-discovery for it)")
    parser.add_argument('--list-ports', action='store_true',
                         help="Just scan and identify connected devices, then exit (no GUI)")
    return parser.parse_args()

def main():
    """Main entry point"""
    args = parse_args()

    if args.list_ports:
        config = Config()
        found, candidates = discover_devices(config)
        print("\nResult:")
        print(f"  Teensy DAQ:  {found.get('teensy', 'not found')}")
        print(f"  Motor ctrl:  {found.get('motor', 'not found')}")
        return

    app = Application()

    try:
        if app.initialize(motor_port_override=args.motor_port, data_port_override=args.data_port):
            app.run()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        app.cleanup()

if __name__ == "__main__":
    main()