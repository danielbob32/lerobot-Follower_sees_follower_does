import serial
import time
import struct

# --- CONFIGURATION ---
SERIAL_PORT = 'COM7'  # <--- Check your port!
BAUD_RATE = 1000000  # Default for Waveshare/ST3215


def calculate_checksum(data_list):
    total = sum(data_list)
    return (~total) & 0xFF


def ping_id(ser, servo_id):
    """
    Sends a PING packet to a specific ID.
    Returns True if that ID sends a response.
    """
    # Packet: Header(2) + ID(1) + Len(1) + Instr(1) + Checksum(1)
    # Instruction 1 is PING
    length = 2
    instruction = 0x01
    checksum = calculate_checksum([servo_id, length, instruction])

    packet = struct.pack('<BBBBBB', 0xFF, 0xFF, servo_id, length, instruction, checksum)

    # Clear any old data in the buffer
    ser.reset_input_buffer()

    # Send Ping
    ser.write(packet)

    # Wait for response (short wait is fine for PING)
    time.sleep(0.02)
    # Check if we got data back
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        # A valid packet starts with 0xFF 0xFF and has at least 6 bytes
        if len(response) >= 6 and response[0] == 0xFF and response[1] == 0xFF:
            return True
    return False


if __name__ == "__main__":
    try:
        print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)

        print("\n--- 🔍 STARTING SERVO SCAN (IDs 0 to 20) ---")
        print("Scanning... (This takes about 5 seconds)")

        found_count = 0

        # Scan IDs 0 through 20 (Common range)
        # Change range(21) to range(254) if you want to scan EVERYTHING
        for i in range(21):
            # Print a dot to show it's working
            print(f".", end='', flush=True)

            if ping_id(ser, i):
                print(f"\n✅ FOUND SERVO AT ID: {i}")
                found_count += 1

            # Small delay to prevent bus collisions
            time.sleep(0.02)

        print(f"\n\n--- SCAN COMPLETE ---")
        print(f"Total Servos Found: {found_count}")

        if found_count == 0:
            print("❌ No servos found. Check Power (12V) and USB connection.")
        elif found_count == 1:
            print("⚠️ Only 1 servo found. Check the cable to the 2nd motor.")

        ser.close()

    except serial.SerialException:
        print(f"\n❌ Error: Could not open {SERIAL_PORT}. Is it being used by another app?")
    except Exception as e:
        print(f"\n❌ Error: {e}")
