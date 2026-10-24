; RepRap G-Code for Duet 3
; 20 mm x 20 mm square
; Each side has a nominal travel time of 2 seconds.
; Feedrate: 600 mm/min = 10 mm/s
; No G94 command used.
;
; IMPORTANT:
; - This example assumes the print starts at X=80, Y=40 and Z=0.040.
; - Confirm these coordinates are safe and appropriate for your setup before running.
; - The start move is a rapid/travel move; the four square sides are G1 moves.
; - Acceleration and cornering mean actual side times may not be exactly 2 seconds.
; - M118 markers follow the style in the supplied Duet 3 file.

; --- SETUP ---
G21 ; Set units to millimeters
G90 ; Absolute positioning
M400 ; Wait for queued moves to finish

; --- MOVE TO SQUARE START ---
G0 X80 Y40 Z0.040 F6000 ; Move to starting corner (verify safe position)
M118 P0 S"0" ; Sync timer: start of square

; --- 20 mm SQUARE: 2 s NOMINAL PER SIDE ---
G1 X100 Y40 Z0.040 F600 ; Side 1: 20 mm at 10 mm/s
M118 P0 S"1" ; Sync timer: corner 1

G1 X100 Y60 Z0.040 F600 ; Side 2: 20 mm at 10 mm/s
M118 P0 S"2" ; Sync timer: corner 2

G1 X80 Y60 Z0.040 F600 ; Side 3: 20 mm at 10 mm/s
M118 P0 S"3" ; Sync timer: corner 3

G1 X80 Y40 Z0.040 F600 ; Side 4: 20 mm at 10 mm/s
M118 P0 S"4" ; Sync timer: square complete

M400 ; Wait for queued moves to finish
