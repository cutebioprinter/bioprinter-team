#!/usr/bin/env python3

import sys
import math
import os

filament_area = math.pi * (0.5 ** 2)  # For 1.75mm filament

gcode_commands = []

# --- Sync settings ---
sync_interval = 5.0  # seconds
time_since_last_sync = 0.0
sync_counter = 0


def flowrate_to_int(flowrate):
    return int(flowrate * (2**14))  # FIXED: use ** instead of ^


def get_pump_command(channel, flowrate64):
    if flowrate64 > 4095:
        flowrate64 = 4095
    command = 0x80
    command |= (channel << 13) & 0x6000
    command |= (flowrate64 << 1) & 0x1F00
    command |= flowrate64 & 0x007F
    return command.to_bytes(2, 'big')


# modify file in-place
with open(sys.argv[1], "r", encoding="utf-8") as f:
    gcode_lines = f.readlines()
    current_flowrate = 0  # mm^3/s
    current_feedrate = 0
    current_position = (-1, -1, -1)

    for line in gcode_lines:
        line = line.split(";", 1)[0]  # Remove comments
        stripped = line.strip()

        if stripped.startswith("G1") or stripped.startswith("G0"):
            x, y, z = current_position
            e = 0
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

            distance = math.dist(current_position, (x, y, z))

            # --- Time calculation ---
            try:
                dt = distance / (f / 60) if f > 0 else 0
            except:
                dt = 0

            time_since_last_sync += dt

            # --- Insert sync commands (robust version) ---
            while time_since_last_sync >= sync_interval:
                gcode_commands.append(f'M118 S"@sync {sync_counter}"\n')
                sync_counter += 1
                time_since_last_sync -= sync_interval

            # --- Flowrate calculation ---
            try:
                flowrate = filament_area * e * (f / 60) / distance
            except:
                flowrate = filament_area * (f / 60) if e != 0 else 0

            if abs(flowrate - current_flowrate) > (0.0001 + 0.03 * current_flowrate):
                gcode_commands.append("G4\n")
                gcode_commands.append(
                    f'M118 S"@pump_pressure 0 {flowrate_to_int(flowrate)}"\n'
                )
                current_flowrate = flowrate

            # Remove E from movement command
            parts = [p for p in stripped.split() if not p.startswith("E")]
            gcode_commands.append(" ".join(parts) + "\n")

            current_position = (x, y, z)
            current_feedrate = f

        # --- Tool changes ---
        elif stripped.startswith("T") and stripped[1:].isdigit():
            channel = int(stripped[1])
            gcode_commands.append(stripped + "\n")

        # --- Pump pressure passthrough ---
        elif stripped.startswith("@pump_pressure"):
            gcode_commands.append("G4\n")
            gcode_commands.append(f'M118 S"{stripped}"\n')

        elif len(stripped) > 0:
            gcode_commands.append(stripped + "\n")

    # Dump slicer environment variables
    for item, value in os.environ.items():
        if item.startswith("SLIC3R"):
            gcode_commands.append(f"\n ;;; {item}: {value}\n")

with open(sys.argv[1], "w", encoding="utf-8") as f:
    f.writelines(gcode_commands)
