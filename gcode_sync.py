# This script continuously monitors serial communications with a 3D printer and pressure pump, 
# updating pressure settings while tracking sequence counters from M118 messages. 
# It uses a background thread and a kinematic G-code engine to interpolate printer movements, 
# calculating and printing the current $(X, Y, Z)$ position 
# alongside a predicted position 5 seconds into the future every 0.5 seconds.

#!/usr/bin/env python3

import serial
import sys
import time
import math
import re
import threading

printer_port = "/dev/ttyACM0"
printer_bps = 115200
pump_port = "/dev/ttyS0"
pump_bps = 9600

if "--printer-port" in sys.argv:
    printer_port = sys.argv[sys.argv.index("--printer-port") + 1]
if "--printer-bps" in sys.argv:
    printer_bps = int(sys.argv[sys.argv.index("--printer-bps") + 1])
if "--pump-port" in sys.argv:
    pump_port = sys.argv[sys.argv.index("--pump-port") + 1]
if "--pump-bps" in sys.argv:
    pump_bps = int(sys.argv[sys.argv.index("--pump-bps") + 1])

# Global state to share between serial reader and position tracker
state_lock = threading.Lock()
current_seq_num = None
seq_timestamp = None

class GCodeKinematicsEngine:
    def __init__(self):
        self.moves = [] # List of tuples: (start_pos, end_pos, speed_mm_s, total_dist, move_duration)
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_feedrate = 1000.0 / 60.0 # Default feedrate in mm/s

    def parse_gcode_file(self, filepath):
        """Parses a pre-sliced G-code file to extract line segments and execution timing."""
        gcode_re = re.compile(r'([X-ZX-ZFE])(-?\d+\.?\d*)')
        current_pos = [0.0, 0.0, 0.0]
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.split(';')[0].strip() # Strip comments
                if not line:
                    continue
                
                parts = line.split()
                cmd = parts[0].upper()
                
                if cmd in ('G0', 'G1'):
                    new_pos = list(current_pos)
                    feedrate = self.current_feedrate
                    
                    for part in parts[1:]:
                        match = gcode_re.match(part)
                        if match:
                            axis, val = match.groups()
                            val = float(val)
                            if axis == 'X': new_pos[0] = val
                            elif axis == 'Y': new_pos[1] = val
                            elif axis == 'Z': new_pos[2] = val
                            elif axis == 'F': feedrate = val / 60.0 # convert mm/min to mm/s
                    
                    # Calculate segment length
                    dx = new_pos[0] - current_pos[0]
                    dy = new_pos[1] - current_pos[1]
                    dz = new_pos[2] - current_pos[2]
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                    
                    duration = dist / feedrate if feedrate > 0 else 0.0
                    
                    self.moves.append({
                        'start': list(current_pos),
                        'end': list(new_pos),
                        'dist': dist,
                        'feedrate': feedrate,
                        'duration': duration
                    })
                    
                    current_pos = new_pos
                    self.current_feedrate = feedrate

    def get_position_at_elapsed_time(self, elapsed_seconds):
        """Calculates exact (X, Y, Z) coordinate based on cumulative movement time."""
        accumulated_time = 0.0
        
        for move in self.moves:
            if accumulated_time + move['duration'] >= elapsed_seconds:
                # Interpolate inside this line segment
                time_in_move = elapsed_seconds - accumulated_time
                fraction = time_in_move / move['duration'] if move['duration'] > 0 else 1.0
                
                x = move['start'][0] + fraction * (move['end'][0] - move['start'][0])
                y = move['start'][1] + fraction * (move['end'][1] - move['start'][1])
                z = move['start'][2] + fraction * (move['end'][2] - move['start'][2])
                return (x, y, z)
            
            accumulated_time += move['duration']
        
        # Return the final destination if elapsed time exceeds the total path duration
        if self.moves:
            return tuple(self.moves[-1]['end'])
        return (0.0, 0.0, 0.0)

def position_tracker_thread():
    """Outputs current and +5 sec positions every 0.5s once sequence sync begins."""
    engine = GCodeKinematicsEngine()
    # If using a pre-parsed G-code file for trajectory lookup, load it here:
    # engine.parse_gcode_file("print_job.gcode")

    while True:
        time.sleep(0.5)
        
        with state_lock:
            seq = current_seq_num
            t_recv = seq_timestamp
            
        if seq is None or t_recv is None:
            continue # Wait for the first M118 message
            
        # Time elapsed since the last M118 message arrived
        elapsed_since_m118 = time.time() - t_recv
        
        # Calculate coordinates using time projection
        # Note: Replace 'elapsed_since_m118' with total estimated job runtime if matching absolute G-code time
        curr_pos = engine.get_position_at_elapsed_time(elapsed_since_m118)
        future_pos = engine.get_position_at_elapsed_time(elapsed_since_m118 + 5.0)
        
        print(f"[Seq #{seq}] T+{elapsed_since_m118:.2f}s | "
              f"Current Position: X={curr_pos[0]:.2f}, Y={curr_pos[1]:.2f}, Z={curr_pos[2]:.2f} | "
              f"Future (+5s): X={future_pos[0]:.2f}, Y={future_pos[1]:.2f}, Z={future_pos[2]:.2f}")

def send_pump_command(ser, channel, pressure):
    if pressure > 2068:
        print(f"Requested pressure of {pressure} too high, Pump Command: channel {channel} at 2068 mbar")
        pressure = 2068
    else:
        print(f"Pump Command: channel {channel} at {pressure} mbar")
    
    command_to_send = 0x80 # mask 0ccxxxxx 1xxxxxxx
    command_to_send |= (channel << 13) & 0x03
    command_to_send |= (pressure << 1) & 0x1F00
    command_to_send |= pressure & 0x007F
    ser.write(command_to_send.to_bytes(2, byteorder='big'))

def main():
    global current_seq_num, seq_timestamp
    
    # Start background position reporting loop
    tracker = threading.Thread(target=position_tracker_thread, daemon=True)
    tracker.start()

    with serial.Serial(printer_port, printer_bps) as printer, serial.Serial(pump_port, pump_bps, timeout=4) as pump:
        while True:
            if printer.in_waiting > 0:
                try:
                    raw_msg = printer.readline()
                    msg = raw_msg.decode().strip()
                    print(f"Printer Message: {msg}")
                    
                    # Handle pump pressure messages
                    if msg.startswith("@pump_pressure"):
                        words = msg.split()
                        send_pump_command(pump, int(words[1]), int(words[2]))
                    
                    # Handle incoming numeric M118 sequence signals (e.g., "0", "1", "2" or "M118 0")
                    numbers = re.findall(r'\b\d+\b', msg)
                    if numbers:
                        seq_val = int(numbers[0])
                        with state_lock:
                            current_seq_num = seq_val
                            seq_timestamp = time.time()
                            
                except Exception as e:
                    print(f"Error retrieving line from printer: {e}")
                    continue

if __name__ == "__main__":
    main()
