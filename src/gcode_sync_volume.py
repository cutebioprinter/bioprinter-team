#!/usr/bin/env python3
"""
Volume-based G-code lookahead and three-channel pressure control.

Core behavior:
- Uses M118 sync markers to align the live printer with the parsed G-code timeline.
- Tracks the printer's position using G0/G1 movement timing.
- Computes a future position after a configurable amount of DEPOSITED volume.
  Travel moves (G0, and moves without positive extrusion) update position/time
  but add no deposited volume.
- Evaluates a user-provided gradient at that future position.
- Converts the three mixing fractions to target flow rates, then uses a
  calibrated linear electrical-analogy model to calculate channel pressures.
- Optionally sends pressures to the pump. Hardware operation is enabled by
  default; use --dry-run to test without opening serial ports.

IMPORTANT:
1. Set R1, R2, R3, and RN to calibrated hydraulic resistance values in
   consistent pressure/flow units.
2. Set TOTAL_FLOW_RATE in the same volume/time units used for the resistances.
3. Implement gradient(x, y, z) for your actual stiffness/mixing-ratio field.
4. The G-code must contain extrusion values (E) if auto-detection is used.
   This parser supports standard M82 (absolute E), M83 (relative E), and G92 E.
5. Validate on a dry run before connecting the pump. The pump protocol and
   channel encoding should be checked against the hardware documentation.
"""

import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import serial
except ImportError:
    serial = None


# ----------------------------- User configuration -----------------------------

DEFAULT_GCODE_FILE = "print_job.gcode"

PRINTER_PORT = "/dev/ttyACM0"
PRINTER_BPS = 115200
PUMP_PORT = "/dev/ttyS0"
PUMP_BPS = 9600

# Set these from calibration. They must use consistent units:
# pressure = flow_rate * resistance.
R1 = 40.0
R2 = 10.0
R3 = 10.0
RN = 10.0

# Constant total flow rate while depositing. Units must match your resistance
# calibration. For example, if flow is mm^3/s, resistances must be pressure
# units per (mm^3/s).
TOTAL_FLOW_RATE = 1.0

# Physical volume from mixing point to nozzle that should be displaced before
# the newly commanded composition reaches the nozzle.
DEAD_VOLUME = 25.0

# How often the tracker reports and updates the pressure target.
UPDATE_INTERVAL_S = 0.5

# Pump command maximum from the existing protocol.
MAX_PRESSURE = 2068


# ----------------------------- Gradient function ------------------------------

def gradient(x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Replace this example with the project's actual stiffness -> mixing-ratio
    function. Return three nonnegative fractions whose sum is 1.

    This default is only a placeholder gradient and should not be used for a
    real print without replacing it.
    """
    a = max(0.0, min(1.0, (x - 60.0) / 50.0))
    b = max(0.0, min(1.0, (y - 22.0) / 50.0))
    c = max(0.0, min(1.0, 1.0 - a - b))
    total = a + b + c
    if total <= 0.0:
        return (1.0, 0.0, 0.0)
    return (a / total, b / total, c / total)


# ------------------------------ Pressure model -------------------------------

def flowrates_to_pressures(
    q1: float,
    q2: float,
    q3: float,
    r1: float = R1,
    r2: float = R2,
    r3: float = R3,
    rn: float = RN,
) -> tuple[float, float, float]:
    """
    Three-branch linear resistance model, analogous to the electrical circuit.

    Shared-node pressure:
        Pn = RN * (Q1 + Q2 + Q3)

    Branch pressures:
        Pi = Qi * Ri + Pn

    This assumes the calibrated hydraulic system follows this linear model,
    with consistent units and the same flow sign convention for all channels.
    """
    if min(r1, r2, r3, rn) < 0:
        raise ValueError("Resistance values must be nonnegative.")
    q_total = q1 + q2 + q3
    pn = q_total * rn
    return q1 * r1 + pn, q2 * r2 + pn, q3 * r3 + pn


def normalized_ratio(values) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError("gradient(x, y, z) must return exactly three values.")
    vals = [float(v) for v in values]
    if any(not math.isfinite(v) or v < 0 for v in vals):
        raise ValueError("Mixing ratios must be finite and nonnegative.")
    total = sum(vals)
    if total <= 0:
        raise ValueError("At least one mixing ratio must be positive.")
    return tuple(v / total for v in vals)


# -------------------------------- G-code data --------------------------------

@dataclass
class Move:
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    start_time: float
    duration: float
    depositing: bool
    start_volume: float
    end_volume: float


class GCodeKinematicsEngine:
    """Parse G0/G1 movement timing and cumulative deposited volume."""

    AXIS_RE = re.compile(r"([XYZEF])(-?(?:\d+(?:\.\d*)?|\.\d+))", re.I)
    M118_RE = re.compile(r"^\s*M118\b(.*)$", re.I)

    def __init__(self, total_flow_rate: float):
        if total_flow_rate <= 0:
            raise ValueError("TOTAL_FLOW_RATE must be positive.")
        self.total_flow_rate = float(total_flow_rate)
        self.moves: list[Move] = []
        self.seq_times: dict[int, float] = {}
        self.seq_volumes: dict[int, float] = {}
        self.total_wall_time = 0.0
        self.total_print_volume = 0.0
        self.final_pos = (0.0, 0.0, 0.0)

    def parse_gcode_file(self, filepath: str) -> None:
        self.moves.clear()
        self.seq_times.clear()
        self.seq_volumes.clear()

        pos = [0.0, 0.0, 0.0]
        feedrate = 1000.0 / 60.0  # mm/s
        wall_time = 0.0
        print_volume = 0.0
        e_position = 0.0
        e_relative = False  # Standard G-code default: absolute extrusion (M82).

        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            for raw_line in f:
                # Preserve quoted M118 payloads but remove semicolon comments.
                line = raw_line.split(";", 1)[0].strip()
                if not line:
                    continue

                parts = line.split()
                cmd = parts[0].upper()

                if cmd == "M82":
                    e_relative = False
                    continue
                if cmd == "M83":
                    e_relative = True
                    continue
                if cmd == "G92":
                    for token in parts[1:]:
                        m = self.AXIS_RE.fullmatch(token)
                        if m and m.group(1).upper() == "E":
                            e_position = float(m.group(2))
                    continue

                if cmd == "M118":
                    # Expected examples: M118 S"@sync 12" or M118 12.
                    payload = line[len(parts[0]):].strip()
                    sync_match = re.search(r"@sync\s+#?\s*(\d+)", payload, re.I)
                    if not sync_match:
                        # Accept a simple numeric M118 payload, but not the
                        # command number 118 itself.
                        plain = re.fullmatch(r'(?:S)?["\']?\s*(\d+)\s*["\']?', payload)
                        if plain:
                            seq = int(plain.group(1))
                        else:
                            continue
                    else:
                        seq = int(sync_match.group(1))
                    self.seq_times[seq] = wall_time
                    self.seq_volumes[seq] = print_volume
                    continue

                if cmd not in ("G0", "G00", "G1", "G01"):
                    continue

                new_pos = list(pos)
                next_feedrate = feedrate
                e_value = None
                for token in parts[1:]:
                    m = self.AXIS_RE.fullmatch(token)
                    if not m:
                        continue
                    axis = m.group(1).upper()
                    value = float(m.group(2))
                    if axis == "X":
                        new_pos[0] = value
                    elif axis == "Y":
                        new_pos[1] = value
                    elif axis == "Z":
                        new_pos[2] = value
                    elif axis == "F":
                        if value > 0:
                            next_feedrate = value / 60.0
                    elif axis == "E":
                        e_value = value

                delta_e = 0.0
                if e_value is not None:
                    if e_relative:
                        delta_e = e_value
                        e_position += e_value
                    else:
                        delta_e = e_value - e_position
                        e_position = e_value

                dx = new_pos[0] - pos[0]
                dy = new_pos[1] - pos[1]
                dz = new_pos[2] - pos[2]
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                duration = distance / next_feedrate if next_feedrate > 0 else 0.0

                # Standard filament-style convention: positive E change on a
                # G1 is deposition. Travel and retract moves add no volume.
                depositing = cmd in ("G1", "G01") and delta_e > 1e-9
                start_volume = print_volume
                if depositing:
                    print_volume += self.total_flow_rate * duration

                self.moves.append(Move(
                    start=tuple(pos),
                    end=tuple(new_pos),
                    start_time=wall_time,
                    duration=duration,
                    depositing=depositing,
                    start_volume=start_volume,
                    end_volume=print_volume,
                ))

                wall_time += duration
                pos = new_pos
                feedrate = next_feedrate

        self.total_wall_time = wall_time
        self.total_print_volume = print_volume
        self.final_pos = tuple(pos)

        if not self.moves:
            raise ValueError(f"No G0/G1 moves found in {filepath!r}.")

    def get_position_at_wall_time(self, elapsed: float) -> tuple[float, float, float]:
        if elapsed <= 0:
            return self.moves[0].start
        for move in self.moves:
            end_time = move.start_time + move.duration
            if move.start_time <= elapsed <= end_time:
                if move.duration <= 0:
                    return move.end
                frac = max(0.0, min(1.0, (elapsed - move.start_time) / move.duration))
                return tuple(
                    move.start[i] + frac * (move.end[i] - move.start[i])
                    for i in range(3)
                )
        return self.final_pos

    def get_position_after_volume(
        self, current_wall_time: float, volume_ahead: float
    ) -> tuple[float, float, float]:
        """
        Find the position after depositing `volume_ahead` additional volume.

        The simulated position/time advances through all moves, including
        travel, but only deposition moves consume the requested volume budget.
        If the requested volume exceeds the remaining print, return final_pos.
        """
        if volume_ahead <= 0:
            return self.get_position_at_wall_time(current_wall_time)

        for move in self.moves:
            move_end_time = move.start_time + move.duration
            if move_end_time < current_wall_time:
                continue

            segment_start_time = max(current_wall_time, move.start_time)
            if move.duration <= 0:
                continue
            segment_fraction_start = (segment_start_time - move.start_time) / move.duration
            segment_fraction_start = max(0.0, min(1.0, segment_fraction_start))
            segment_start_pos = tuple(
                move.start[i] + segment_fraction_start * (move.end[i] - move.start[i])
                for i in range(3)
            )
            remaining_duration = max(0.0, move_end_time - segment_start_time)

            if not move.depositing:
                # Travel does not consume volume, but position still changes.
                continue

            remaining_volume = self.total_flow_rate * remaining_duration
            if volume_ahead <= remaining_volume:
                seconds_into_segment = volume_ahead / self.total_flow_rate
                fraction = (segment_start_time + seconds_into_segment - move.start_time) / move.duration
                fraction = max(0.0, min(1.0, fraction))
                return tuple(
                    move.start[i] + fraction * (move.end[i] - move.start[i])
                    for i in range(3)
                )
            volume_ahead -= remaining_volume

        return self.final_pos


# ------------------------------ Serial tracking ------------------------------

state_lock = threading.Lock()
current_seq_num: Optional[int] = None
seq_timestamp: Optional[float] = None


def send_pump_command(ser, channel: int, pressure: float) -> None:
    pressure_int = int(round(max(0.0, min(MAX_PRESSURE, pressure))))
    if pressure > MAX_PRESSURE:
        print(f"Requested pressure {pressure:.2f}; clamped to {MAX_PRESSURE} mbar")
    if pressure < 0:
        print(f"Requested negative pressure {pressure:.2f}; clamped to 0 mbar")

    # Preserves the command encoding from the supplied original script.
    # Verify channel bit placement against the pump's protocol before hardware use.
    command = 0x80
    command |= (channel << 13) & 0x03
    command |= (pressure_int << 1) & 0x1F00
    command |= pressure_int & 0x007F
    ser.write(command.to_bytes(2, byteorder="big"))
    print(f"Pump command: channel {channel}, pressure {pressure_int} mbar")


def parse_sync_sequence(message: str) -> Optional[int]:
    """Only recognize explicit sync messages; ignore unrelated numeric messages."""
    match = re.search(r"@sync\s+#?\s*(\d+)", message, re.I)
    if match:
        return int(match.group(1))
    # Support printers that echo a bare numeric payload from M118.
    match = re.fullmatch(r'\s*(?:echo:\s*)?(?:M118\s+)?["\']?(\d+)["\']?\s*', message, re.I)
    return int(match.group(1)) if match else None


def position_tracker_thread(args, pump_holder, pump_ready):
    engine = GCodeKinematicsEngine(args.flow_rate)
    engine.parse_gcode_file(args.gcode)
    print(
        f"Loaded {args.gcode}: {len(engine.moves)} moves, "
        f"{engine.total_print_volume:.3f} modeled deposited volume, "
        f"{len(engine.seq_times)} sync markers."
    )

    last_seq = None
    last_real_time = None
    time_multiplier = 1.0
    last_pressure_tuple = None

    while True:
        time.sleep(args.update_interval)
        with state_lock:
            seq = current_seq_num
            t_recv = seq_timestamp

        if seq is None or t_recv is None:
            continue

        if seq not in engine.seq_times:
            print(f"Received sync #{seq}, but it is not in the parsed G-code.")
            continue

        if seq != last_seq and last_seq is not None:
            real_delta = t_recv - last_real_time
            gcode_delta = engine.seq_times[seq] - engine.seq_times.get(last_seq, engine.seq_times[seq])
            if real_delta > 0 and gcode_delta > 0:
                estimate = gcode_delta / real_delta
                time_multiplier = 0.5 * time_multiplier + 0.5 * estimate

        last_seq = seq
        last_real_time = t_recv

        base_job_time = engine.seq_times[seq]
        elapsed = max(0.0, time.time() - t_recv)
        total_job_time = base_job_time + elapsed * time_multiplier

        curr_pos = engine.get_position_at_wall_time(total_job_time)
        future_pos = engine.get_position_after_volume(total_job_time, args.dead_volume)
        ratio = normalized_ratio(gradient(*future_pos))
        target_flows = tuple(v * args.flow_rate for v in ratio)
        pressures = flowrates_to_pressures(*target_flows)

        print(
            f"[Sync #{seq} | clock scale {time_multiplier:.2f}] "
            f"t={total_job_time:.2f}s | "
            f"current=({curr_pos[0]:.2f}, {curr_pos[1]:.2f}, {curr_pos[2]:.2f}) | "
            f"volume-lookahead={args.dead_volume:.3f} | "
            f"future=({future_pos[0]:.2f}, {future_pos[1]:.2f}, {future_pos[2]:.2f}) | "
            f"ratio={tuple(round(v, 4) for v in ratio)} | "
            f"flows={tuple(round(v, 4) for v in target_flows)} | "
            f"pressures={tuple(round(v, 2) for v in pressures)}"
        )

        if args.dry_run or not pump_ready.is_set():
            continue

        if pressures != last_pressure_tuple:
            pump = pump_holder[0]
            for channel, pressure in enumerate(pressures):
                send_pump_command(pump, channel, pressure)
            last_pressure_tuple = pressures


def main():
    global current_seq_num, seq_timestamp

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcode", default=DEFAULT_GCODE_FILE)
    parser.add_argument("--printer-port", default=PRINTER_PORT)
    parser.add_argument("--printer-bps", type=int, default=PRINTER_BPS)
    parser.add_argument("--pump-port", default=PUMP_PORT)
    parser.add_argument("--pump-bps", type=int, default=PUMP_BPS)
    parser.add_argument("--flow-rate", type=float, default=TOTAL_FLOW_RATE,
                        help="constant total flow rate while depositing")
    parser.add_argument("--dead-volume", type=float, default=DEAD_VOLUME,
                        help="volume to look ahead, in the same volume units as flow-rate*time")
    parser.add_argument("--update-interval", type=float, default=UPDATE_INTERVAL_S)
    parser.add_argument("--dry-run", action="store_true",
                        help="do not open pump serial port or send pump commands")
    args = parser.parse_args()

    if args.flow_rate <= 0:
        parser.error("--flow-rate must be positive")
    if args.dead_volume < 0:
        parser.error("--dead-volume cannot be negative")
    if serial is None and not args.dry_run:
        parser.error("pyserial is not installed. Install it or use --dry-run.")

    pump_holder = [None]
    pump_ready = threading.Event()

    # In dry-run mode, no serial devices are opened.
    if args.dry_run:
        tracker = threading.Thread(
            target=position_tracker_thread,
            args=(args, pump_holder, pump_ready),
            daemon=True,
        )
        tracker.start()
        print("DRY RUN: serial ports are not opened; waiting for sync messages is disabled.")
        print("For offline parser testing, use the --self-test option in a future extension.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            return

    try:
        with serial.Serial(args.printer_port, args.printer_bps, timeout=1) as printer, \
             serial.Serial(args.pump_port, args.pump_bps, timeout=1) as pump:
            pump_holder[0] = pump
            pump_ready.set()
            tracker = threading.Thread(
                target=position_tracker_thread,
                args=(args, pump_holder, pump_ready),
                daemon=True,
            )
            tracker.start()
            print("Connected. Waiting for explicit @sync messages from the printer.")

            while True:
                raw = printer.readline()
                if not raw:
                    continue
                message = raw.decode(errors="replace").strip()
                if message:
                    print(f"Printer: {message}")
                seq = parse_sync_sequence(message)
                if seq is not None:
                    with state_lock:
                        current_seq_num = seq
                        seq_timestamp = time.time()
    except KeyboardInterrupt:
        print("Stopping.")
    except Exception as exc:
        print(f"Serial/control error: {exc}", file=sys.stderr)
        raise


if __name__ == "__main__":
    main()
