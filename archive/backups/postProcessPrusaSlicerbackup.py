#!/usr/bin/env python3

import sys
import math
import os

filament_area = math.pi * (0.5 ** 2) # For 1.75mm filament, but 0.5^2 / 1.75mm^2, reversing the math for 1.75mm filament

gcode_commands = [] 

def flowrate_to_int(flowrate):
	return int(flowrate*2^14); #match with forward to pump code.
    ##return int((flowrate * resistance) + offset) if flowrate != 0 else 10 # if no flowrate, set to
                                                                          # minimum pressure instead
                                                                          # of turning off

def get_pump_command(channel, flowrate64):
    if flowrate64 > 4095:
        #print(f"Requested pressure of {pressure} too high, Pump Command: channel {channel} at 2068 mbar")
        flowrate64 = 4095
    #else:
        #print(f"Pump Command: channel {channel} at {pressure} mbar")
    command = 0x80 # mask 0ccxxxxx 1xxxxxxx
    command |= (channel << 13) & 0x6000
    command |= (flowrate64 << 1) & 0x1F00
    command |= flowrate64 & 0x007F
    return command.to_bytes(2, 'big')

# Ideally this logic would be passed to the pressure pump

# modify file in-place
with open(sys.argv[1], "r", encoding="utf-8") as f:
    gcode_lines = f.readlines()
    current_flowrate = 0 # mm^3/s
    current_feedrate = 0
    current_position = (-1,-1,-1)
    ## Transform gcode
    for line in gcode_lines:
        line = line.split(";", 1)[0] # Remove all comments

        stripped = line.strip()
        if stripped.startswith("G1") or stripped.startswith("G0"):
            # Extruder position is relative
            x, y, z = current_position
            e = 0 # assume 0 extruder movement
            f = current_feedrate
            for el in stripped.split():
                if el.startswith("X"):
                    x = float(el[1:])
                elif el.startswith("Y"):
                    y = float(el[1:])
                elif el.startswith("Z"):
                    z = float(el[1:])
                elif el.startswith("E"):
                    e = float(el[1:])
                elif el.startswith("F"):
                    f = float(el[1:])
            distance = math.dist(current_position, (x,y,z))

            try:
                flowrate = filament_area * e * (f/60) / distance
            except: # ensure this is correct
                flowrate = filament_area * (f/60) if e != 0 else 0 # if no extruder movement, 0 flowrate
			
            if abs(flowrate - current_flowrate) > (0.0001+0.03*current_flowrate): # if new flowrate is significantly different
                gcode_commands.append("G4\n") # This line may not be necessary when printing from SD
                gcode_commands.append(f"M118 S\"@pump_pressure 0 {flowrate_to_int(flowrate)}\"\n")
                #press_cmd = get_pump_command(0, flowrate_to_pressure(flowrate))  # for later
                #gcode_commands.append(f"M260.2 P1 B{press_cmd[0]}:{press_cmd[1]}")
                current_flowrate = flowrate
            #else:
                #gcode_commands.append(";\n")

            parts = [p for p in stripped.split() if not p.startswith("E")] ## Make more ideomatic
            gcode_commands.append(" ".join(parts) + "\n")
            current_position = (x,y,z)
            current_feedrate = f
        # --- Track tool changes (T0/T1/T2) to set channel bits ---
        elif stripped.startswith("T") and stripped[1].isdigit():
            channel = int(stripped[1])
            gcode_commands.append(stripped + "\n")
        elif stripped.startswith("@pump_pressure"): ## special command
            gcode_commands.append("G4\n") # Same as 14 lines above
            gcode_commands.append(f"M118 S\"{stripped}\"\n")
        elif len(stripped) > 0:
            gcode_commands.append(stripped + "\n")
    # Dump slicer environment variables
    for item, value in os.environ.items():
        if item.startswith("SLIC3R"):
            gcode_commands.append(f"\n ;;; {item}: {value}\n")

with open(sys.argv[1], "w", encoding="utf-8") as f:
    f.writelines(gcode_commands)
