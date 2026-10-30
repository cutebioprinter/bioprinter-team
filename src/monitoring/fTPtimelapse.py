#!/usr/bin/env python3

import serial
import sys
import os
import cv2

printer_port = "/dev/ttyACM0"
printer_bps = 115200
pump_port = "/dev/ttyS0"
pump_bps = 9600
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
frame_counter = 0
PHOTO_FOLDER = '/home/pi/timelapse_frames'

## Write logic to determine this from slicer
resistance = 3576
offset = 816
flowconstant = 1.1 #multiplier to place more bioink
def flowrate_to_pressure(flowrate):
	return int(((flowrate/2^14 * resistance) + offset)*flowconstant) if flowrate != 0 else 10 # if no flowrate, set to
 

if "--printer-port" in sys.argv:
    printer_port = sys.argv[sys.argv.index("--printer-port") + 1]
if "--printer-bps" in sys.argv:
    printer_bps = int(sys.argv[sys.argv.index("--printer-bps") + 1])
if "--pump-port" in sys.argv:
    pum_port = sys.argv[sys.argv.index("--pump-port") + 1]
if "--pump-bps" in sys.argv:
    pump_bps = int(sys.argv[sys.argv.index("--pump-bps") + 1])

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
    ser.write(command_to_send.to_bytes(2))

with serial.Serial(printer_port, printer_bps) as printer, serial.Serial(pump_port, pump_bps, timeout=4) as pump:
    while True:
        if printer.in_waiting > 0:
            try:
                msg = printer.readline() # might not be very noise tolerant
                print("Printer Message: " + msg.decode())
                if msg.startswith(b"@pump_pressure"):
                    words = msg.split()
                    send_pump_command(pump, int(words[1]), flowrate_to_pressure(int(words[2])))
                if msg.startswith(b"@pic"):
                    print("taking pic!")
                    ret, frame = cap.read()
                    if ret:
                        print("saving pic!")
                        filename = os.path.join(PHOTO_FOLDER, f"frame_{frame_counter:04d}.jpg")
                        cv2.imwrite(filename, frame)
                        frame_counter += 1
            except:
                print("Error retrieving line from printer. Retrying.")
                continue
            #finally:
                #cap.release()

