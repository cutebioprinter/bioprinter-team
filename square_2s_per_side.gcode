; square_2s_per_side.gcode
; Example: 20 mm square, nominally 2 seconds per side at 600 mm/min (10 mm/s).
; IMPORTANT:
; - Verify the printer's coordinate system and starting position before running.
; - This file uses absolute XY coordinates and assumes the square starts at X0 Y0.
; - Actual side times may differ due to acceleration, junction planning, and firmware limits.
; - M118 sequence markers are placed at the start and each corner for synchronization.
; - The provided Python parser must extract the sequence number after M118, not the "118" in M118.

G21             ; Units in millimeters
G90             ; Absolute positioning
G94             ; Feedrate in mm/min
M204 S1000      ; Example acceleration setting; verify appropriate value for your printer

G1 X0 Y0 F600   ; Move to square start (this initial move is not one of the four sides)
M118 S"@sync 0"

G1 X20 Y0 F600 ; Side 1: 20 mm at 10 mm/s = nominally 2 s
M118 S"@sync 1"

G1 X20 Y20 F600 ; Side 2
M118 S"@sync 2"

G1 X0 Y20 F600 ; Side 3
M118 S"@sync 3"

G1 X0 Y0 F600 ; Side 4
M118 S"@sync 4"

M400            ; Wait for queued motion to finish
M118 S"@sync 5"
