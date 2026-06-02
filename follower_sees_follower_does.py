import serial
import time
import signal
import sys

from dashboard import state as dash_state
from dashboard import server as dash_server
from dashboard import control as dash_control

# --- CONFIGURATION ---
LEADER_PORT = 'COM6'  # Check port
FOLLOWER_PORT = 'COM7'  # Check port
BAUDRATE = 1000000

LEADER_IDS = [7, 8, 9, 10, 11, 12]
FOLLOWER_IDS = [1, 2, 3, 4, 6, 5]

DIRECTIONS = [1, 1, 1, 1, 1, 1]
# --- END CONFIGURATION ---

# --- REGISTERS ---
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56


def calculate_checksum(packet):
    s = sum(packet[2:])
    return (~s) & 0xFF


def write_byte(ser, servo_id, address, value):
    packet = [0xFF, 0xFF, servo_id, 4, 0x03, address, value]
    packet.append(calculate_checksum(packet))
    ser.write(bytearray(packet))


def read_position_robust(ser, servo_id):
    ser.reset_input_buffer()
    # Fast read: 1 attempt only to keep loop speed high.
    # If missed, we skip this frame (better than lagging).
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, ADDR_PRESENT_POSITION, 2]
    packet.append(calculate_checksum(packet))
    ser.write(bytearray(packet))
    try:
        r = ser.read(8)
        if len(r) == 8 and r[0] == 0xFF:
            val = (r[6] << 8) | r[5]
            return val % 4096  # Normalize for Delta Calculation
    except:
        pass
    return None


def read_telemetry(ser, servo_id):
    """Read 8 bytes starting at ADDR_PRESENT_POSITION: pos(2), speed(2),
    load(2), voltage(1), temperature(1). Returns a dict of normalized
    values or None on any read failure. Single attempt — same discipline
    as read_position_robust."""
    ser.reset_input_buffer()
    packet = [0xFF, 0xFF, servo_id, 4, 0x02, ADDR_PRESENT_POSITION, 8]
    packet.append(calculate_checksum(packet))
    ser.write(bytearray(packet))
    try:
        # Response: header(2)+id(1)+len(1)+err(1)+data(8)+chk(1) = 14 bytes
        r = ser.read(14)
        if len(r) != 14 or r[0] != 0xFF or r[1] != 0xFF:
            return None
        pos = ((r[6] << 8) | r[5]) % 4096
        speed_raw = (r[8] << 8) | r[7]
        load_raw = (r[10] << 8) | r[9]
        volt_raw = r[11]
        temp_raw = r[12]
        # ST3215 load is 10 bits + direction in bit 10; ignore direction,
        # rescale magnitude 0-1023 to 0-100 (approximate percent).
        load_pct = int((load_raw & 0x3FF) * 100 / 1023)
        return {
            "pos": pos,
            "speed": speed_raw,
            "load": load_pct,
            "volt": volt_raw / 10.0,
            "temp": temp_raw,
        }
    except Exception:
        return None


def sync_write_positions(ser, ids, targets):
    """
    Sends ONE packet to move ALL motors instantly.
    Structure: [Header, ID=0xFE, Len, Instr=0x83, Addr, DataLen, ID1, P1L, P1H, ID2, P2L, P2H...]
    """
    # 0x83 is SYNC WRITE
    # Data Length per motor = 2 bytes (Position L, Position H)
    data_len = 2

    # Calculate total packet length:
    # (ID + Data) * N_Motors + 4 bytes (Addr + Len)
    total_len = ((1 + data_len) * len(ids)) + 4

    packet = [0xFF, 0xFF, 0xFE, total_len, 0x83, ADDR_GOAL_POSITION, data_len]

    for i in range(len(ids)):
        sid = ids[i]
        pos = int(targets[i])

        # Handle negative/large numbers for Infinite Mode
        if pos < 0: pos += 65536

        pL = pos & 0xFF
        pH = (pos >> 8) & 0xFF

        packet.extend([sid, pL, pH])

    packet.append(calculate_checksum(packet))
    ser.write(bytearray(packet))


_interrupted_once = False


def _sigint_handler(signum, frame):
    """Two-stage Ctrl+C. First press: release sim mode, freeze motion,
    keep torque on. Any subsequent press: full shutdown."""
    global _interrupted_once
    if not _interrupted_once:
        _interrupted_once = True
        dash_control.release()
        print("\nMotion frozen. Press Ctrl+C again to exit.")
        return
    print("\nStopping...")
    try:
        for sid in FOLLOWER_IDS:
            write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass
    try: dash_server.stop()
    except Exception: pass
    try: l_ser.close()
    except Exception: pass
    try: f_ser.close()
    except Exception: pass
    sys.exit(0)


signal.signal(signal.SIGINT, _sigint_handler)


try:
    print("Opening High-Speed Ports...")
    l_ser = serial.Serial(LEADER_PORT, BAUDRATE, timeout=0.01)  # Ultra low timeout
    f_ser = serial.Serial(FOLLOWER_PORT, BAUDRATE, timeout=0.01)

    dash_state.init(LEADER_IDS, FOLLOWER_IDS)
    dash_control.init([2048] * 6)  # re-seated after the lock phase
    dash_server.start(host="127.0.0.1", port=8080)

    print("\n--- HIGH SPEED SYNC TELEOP ---")

    # 1. Relax
    for sid in LEADER_IDS: write_byte(l_ser, sid, ADDR_TORQUE_ENABLE, 0)
    for sid in FOLLOWER_IDS: write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)

    print("Move to Sync Position. Press ENTER.")
    input()

    # 2. Init
    prev_leader_pos = [0] * 6
    follower_targets = [0] * 6

    print("Locking...")
    # Initial lock must be individual to read positions safely
    for i in range(6):
        lid = LEADER_IDS[i]
        fid = FOLLOWER_IDS[i]

        lp = read_position_robust(l_ser, lid)
        while lp is None: lp = read_position_robust(l_ser, lid)
        prev_leader_pos[i] = lp
        dash_state.update(lid, pos=lp)

        fp = read_position_robust(f_ser, fid)
        while fp is None: fp = read_position_robust(f_ser, fid)
        follower_targets[i] = fp
        dash_state.update(fid, pos=fp, goal=fp)

        write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)  # Lock

    # Send one initial Sync Write to stiffen them all
    sync_write_positions(f_ser, FOLLOWER_IDS, follower_targets)

    # Re-seat control module with real follower positions.
    dash_control.init(list(follower_targets))

    print(">> Active. Running Sync Write Mode.")

    last_print_time = time.time()
    last_status_targets = list(follower_targets)
    telemetry_cursor = 0

    while True:
        # Loop Variables
        update_needed = False

        # Telemetry cursor picks one motor per iteration to receive the
        # 8-byte block read instead of (for leaders) the 2-byte position
        # read, or (for followers, which have no baseline read) one extra
        # 8-byte read. Full 12-motor refresh in ~24 ms at ~500 Hz loop.
        tele_idx = telemetry_cursor % 12
        tele_is_leader = tele_idx < 6
        tele_arm_idx = tele_idx if tele_is_leader else tele_idx - 6

        # 1. READ PHASE (Leaders)
        for i in range(6):
            lid = LEADER_IDS[i]

            if tele_is_leader and tele_arm_idx == i:
                tele = read_telemetry(l_ser, lid)
                if tele is not None:
                    curr_l = tele["pos"]
                    dash_state.update(lid,
                                      pos=curr_l,
                                      temp=tele["temp"],
                                      volt=tele["volt"],
                                      load=tele["load"],
                                      speed=tele["speed"])
                else:
                    curr_l = None
            else:
                curr_l = read_position_robust(l_ser, lid)
                if curr_l is not None:
                    dash_state.update(lid, pos=curr_l)

            if curr_l is not None:
                # Calculate Delta
                delta = curr_l - prev_leader_pos[i]

                # Wrap-Around Logic
                if delta > 2048:  delta -= 4096
                if delta < -2048: delta += 4096

                if delta != 0:
                    # Update Target
                    prev_leader_pos[i] = curr_l
                    follower_targets[i] += (delta * DIRECTIONS[i])
                    update_needed = True

        # 1b. Follower telemetry (only when the cursor lands on one)
        if not tele_is_leader:
            fid = FOLLOWER_IDS[tele_arm_idx]
            tele = read_telemetry(f_ser, fid)
            if tele is not None:
                dash_state.update(fid,
                                  pos=tele["pos"],
                                  temp=tele["temp"],
                                  volt=tele["volt"],
                                  load=tele["load"],
                                  speed=tele["speed"])

        telemetry_cursor += 1

        # 2. WRITE PHASE — route through control so sim mode can override
        # and safety clamps apply.
        final_targets = dash_control.next_follower_targets(
            list(follower_targets), time.time())
        if dash_control.consume_force_release():
            print("[control] heartbeat lost — released to physical")
            for sid in FOLLOWER_IDS:
                write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
            for i in range(6):
                fid = FOLLOWER_IDS[i]
                fp = read_position_robust(f_ser, fid)
                if fp is not None:
                    final_targets[i] = fp
                    follower_targets[i] = fp
                write_byte(f_ser, fid, ADDR_TORQUE_ENABLE, 1)
            dash_control.init(list(final_targets))
        # In sim mode, final_targets may differ from follower_targets every
        # iteration even without leader motion, so always write.
        sync_write_positions(f_ser, FOLLOWER_IDS, final_targets)
        for i in range(6):
            dash_state.update(FOLLOWER_IDS[i], goal=int(final_targets[i]))

        if time.time() - last_print_time >= 5.0:
            print(f"\n--- STATUS ({time.strftime('%H:%M:%S')}) ---")

            # Build the status string
            for i in range(6):
                fid = FOLLOWER_IDS[i]
                current_pos = int(follower_targets[i])

                # Calculate difference from 5 seconds ago
                diff = current_pos - int(last_status_targets[i])

                # Format nicely: "ID 6: 14500 (+200)"
                sign = "+" if diff >= 0 else ""
                print(f"ID {fid}: {current_pos:<6} ({sign}{diff})", end=" | ")

            print("")

            # Update history for next comparison
            last_status_targets = list(follower_targets)
            last_print_time = time.time()

        # Small delay not needed because read_robust acts as a throttle
        # but a tiny sleep yields CPU to USB driver
        time.sleep(0.002)

except Exception as e:
    print(f"\nUnexpected error: {e}")
    try:
        for sid in FOLLOWER_IDS: write_byte(f_ser, sid, ADDR_TORQUE_ENABLE, 0)
    except Exception:
        pass
    try: dash_server.stop()
    except Exception: pass
    try: l_ser.close()
    except Exception: pass
    try: f_ser.close()
    except Exception: pass
    raise
