#!/usr/bin/env python3

import numpy as np
import re
import serial
import time


# -------- SETTINGS --------
DT = 0.1                 # sampling resolution (seconds)
LOOKAHEAD = 0.5          # seconds ahead
TOTAL_FLOW_SCALE = 1000  # maps (a,b,c) → (flowA, flowB, flowC)

printer_port = "/dev/ttyACM0"
printer_bps = 115200

pump_port = "/dev/ttyS0"
pump_bps = 9600

def flowrate_to_pressure(flowratex,flowratey,flowratez): #may need to add node pressure
    return flowratex*1000,flowratey*1000,flowratez*1000 #need actual algorithm

def flowrate214_to_flowrate(flowrate):
    if flowrate == 0:
        return 10
    return int(flowrate / (2**14))

# -------- GRADIENT FUNCTION --------
def gradient(x, y, z):

    a = np.clip((x-60)/50, 0, 1)
    b = np.clip((y-22)/50, 0, 1)
    c = 1 - a - b
    c = np.clip(c, 0, 1)

    total = a + b + c
    return np.array([a, b, c]) / total


# -------- PARSE + BUILD TRAJECTORY --------
def build_trajectory(filepath):

    time_array = []
    mix_array = []
    flow_array = []
    sync_map = {}

    x = y = z = 0
    feedrate = 1000
    current_time = 0
    flowrate = 0

    with open(filepath, "r") as f:

        for line in f:

            line = line.strip()

            if line.startswith(";") or line == "":
                continue

            # -------- SYNC MARKER --------
            sync_match = re.search(r'M118\s+S"@sync\s+(\S+)"', line)
            if sync_match:
                marker = sync_match.group(1)
                sync_map[marker] = len(time_array)
                continue

            # -------- PUMP COMMAND --------
            pump_match = re.search(r'@pump_pressure\s+\d+\s+([-+]?[0-9]*\.?[0-9]+)', line)
            if pump_match:
                flowrate = flowrate214_to_flowrate(float(pump_match.group(1)))
                
                continue

            # -------- FEEDRATE --------
            f_match = re.search(r'F([-+]?[0-9]*\.?[0-9]+)', line)
            if f_match:
                feedrate = float(f_match.group(1))

            # -------- MOTION --------
            if line.startswith(("G0", "G1")):

                new_x, new_y, new_z = x, y, z

                xm = re.search(r'X([-+]?[0-9]*\.?[0-9]+)', line)
                ym = re.search(r'Y([-+]?[0-9]*\.?[0-9]+)', line)
                zm = re.search(r'Z([-+]?[0-9]*\.?[0-9]+)', line)

                if xm: new_x = float(xm.group(1))
                if ym: new_y = float(ym.group(1))
                if zm: new_z = float(zm.group(1))

                dist = np.sqrt((new_x-x)**2 + (new_y-y)**2 + (new_z-z)**2)
                speed = feedrate / 60
                dt_move = dist / speed if speed > 0 else 0

                steps = max(1, int(np.ceil(dt_move / DT)))

                for i in range(1, steps + 1):

                    alpha = i / steps

                    interp_x = x + alpha * (new_x - x)
                    interp_y = y + alpha * (new_y - y)
                    interp_z = z + alpha * (new_z - z)

                    interp_time = current_time + alpha * dt_move

                    if flowrate == 0:
                        mix = np.array([0, 0, 0])
                    else:
                        mix = gradient(interp_x, interp_y, interp_z)

                    time_array.append(interp_time)
                    mix_array.append(mix)
                    flow_array.append(mix*flowrate)

                current_time += dt_move
                x, y, z = new_x, new_y, new_z


    time_array = np.array(time_array)
    mix_array = np.array(mix_array)
    flow_array = np.array(flow_array)

    return time_array, flow_array, sync_map


# -------- SEND COMMAND --------
def send_pump_command(ser, channel, pressure):
    if pressure > 2068:
	    pressure = 2068
    command_to_send = 0x80 # mask 0ccx xxxx 1xxx xxxx
    command_to_send |= (channel << 13) & 0x6000
    command_to_send |= (pressure << 1) & 0x1F00
    command_to_send |= pressure & 0x007F
    ser.write(command_to_send.to_bytes(2))


# -------- REAL-TIME EXECUTION --------
def run_trajectory(time_array, flow_array, sync_map):

    with serial.Serial(printer_port, printer_bps) as printer, \
         serial.Serial(pump_port, pump_bps, timeout=4) as pump:

        i = 0
        start_time = time.time()
        N = len(time_array)
        lastvals = [10,10,10];
        while True:

            # ---- HANDLE SYNC FROM PRINTER ----
            if printer.in_waiting > 0:
                try:
                    msg = printer.readline().decode().strip()
                    print("Printer:", msg)

                    sync_match = re.search(r'@sync\s+(\S+)', msg)

                    if sync_match:
                        marker = sync_match.group(1)

                        if marker in sync_map:
                            i = sync_map[marker]
                            start_time = time.time()
                            print(f"SYNC → Jumping to index {i}")

                except:
                    print("Error reading printer message")
                    continue

            # ---- TIME-BASED PLAYBACK ----
            t_now = time.time() - start_time
            t_target = t_now + LOOKAHEAD
			
            
            while i < N and time_array[i] <= t_target:
                pressure_array = flowrate_to_pressure(flow_array[i][0],flow_array[i][1],flow_array[i][2])
                
                if (abs(lastvals[0]-pressure_array[0]) > 10):
                    send_pump_command(pump, 0, int(pressure_array[0]))
                    print(0, int(pressure_array[0]))
                    lastvals[0] = int(pressure_array[0])
                if (abs(lastvals[1]-pressure_array[1]) > 10):
                    send_pump_command(pump, 1, int(pressure_array[1]))
                    print(1, int(pressure_array[1]))
                    lastvals[1] = int(pressure_array[1])
                if (abs(lastvals[2]-pressure_array[2]) > 10):
                    send_pump_command(pump, 2, int(pressure_array[2]))
                    print(2, int(pressure_array[2]))
                    lastvals[2] = int(pressure_array[2])
                

                i += 1

            time.sleep(0.01)


# -------- MAIN --------
if __name__ == "__main__":

    gcode_file = "flowrateinttest.gcode"

    print("Building trajectory...")
    time_array, flow_array, sync_map = build_trajectory(gcode_file)

    print("Trajectory length:", len(time_array))
    print("Sync markers:", sync_map)
    print("Running...")
    run_trajectory(time_array, flow_array, sync_map)

