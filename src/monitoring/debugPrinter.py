#!/usr/bin/env python3

import serial
import sys
import time

def send_pump_command(ser, channel, pressure):
    if pressure > 2068:
        print(f"Requested pressure of {pressure} too high! Extruding at max pressure.")
        pressure = 2068
    command_to_send = 0x80 # mask 0ccx xxxx 1xxx xxxx
    command_to_send |= (channel << 13) & 0x6000
    print(command_to_send)
    command_to_send |= (pressure << 1) & 0x1F00
    command_to_send |= pressure & 0x007F
    ser.write(command_to_send.to_bytes(2))
    print(channel)
    print(command_to_send)

with serial.Serial("/dev/ttyACM0", 115200) as prt, serial.Serial("/dev/ttyS0", 9600) as pmp:
    if len(sys.argv) == 1:
        while True:
            cmd = input("Input gcode command: ")
            # consider adding a way to have pressures
            prt.write((cmd + "\n").encode())
            while prt.in_waiting > 0:
                print(prt.readline().decode())
    elif sys.argv[1] == "--purge":
        send_pump_command(pmp, 0, 2068)
        #send_pump_command(pmp, 1, 2068)
        #send_pump_command(pmp, 2, 2068)
        if len(sys.argv) == 2:
            input("Press enter to stop purging")
        elif len(sys.argv) == 3:
            time.sleep(float(sys.argv[2]))
        send_pump_command(pmp, 0, 0)
        send_pump_command(pmp, 1, 0)
        send_pump_command(pmp, 2, 0)
    elif sys.argv[1] == "--sweep-pressures":
        for i in range(8,2069,10):
            send_pump_command(pmp, 0, i)
            time.sleep(0.01)
        time.sleep(1)
        send_pump_command(pmp, 0, 0)
        print("Done")
    elif sys.argv[1] == "--calibrate-flow":
        testlen = 50 # Default test duration in seconds
        test_pressures = [200, 160, 120, 100, 80, 40] # Default test pressures in mbar

        if len(sys.argv) >= 3:
            testlen = int(sys.argv[2])
        if len(sys.argv) > 3:
            test_pressures = []
            for i in range(3,len(sys.argv)):
                test_pressures.append(int(sys.argv[i]))

        print(f"Calibrating Flows at pressures {test_pressures} in mbar with a test duration of {testlen} seconds")

        for press in test_pressures:
            print(f"Extrude at {press}mbar")
            input("Press enter to continue\n")
            send_pump_command(pmp, 0, press) # so far only does channel 0
            time.sleep(testlen)
            send_pump_command(pmp, 0, 0)

        print("Done")
    else:
        print("Incorrect Usage, possible arguments:")
        print("(none): Bring up interactive Serial/Gcode console")
        print("--purge: Purge with max pressure until you hit enter")
        print("--purge n: Purge with max pressure for n seconds")
        print("--sweep-pressures: Sweep pump pressure levels from min to max")
        print("--calibrate-flow: Interactively extrude at pre-set pressure levels for 120 seconds")
        print("--calibrate-flow n: Interactively extrude at pre-set pressure levels for n seconds")
        print("--calibrate-flow n a b c...: Interactively extrude at pressures a b c... (in mbar) for n seconds")

