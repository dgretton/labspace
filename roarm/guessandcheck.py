import serial
import argparse
import threading
import json

# ---- Handles incoming messages from the arm (runs in the background) ----
def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def main():
    global ser

    # ---- Set up the serial connection using the port passed in on the command line ----
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')
    args = parser.parse_args()
    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    # ---- Start the background listener thread ----
    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        # ---- Ask which axis to sweep, and the range/step for that axis ----
        axis = input("Which axis do you want to sweep? (x/y/z): ").strip().lower()
        start = float(input("Start value (mm): "))
        end = float(input("End value (mm): "))
        step = float(input("Step size (mm): "))

        # ---- Ask for the fixed values of the OTHER two axes ----
        # (whichever axis isn't being swept stays parked at these values)
        if axis == 'x':
            fixed_y = float(input("Fixed y value (mm): "))
            fixed_z = float(input("Fixed z value (mm): "))
        elif axis == 'y':
            fixed_x = float(input("Fixed x value (mm): "))
            fixed_z = float(input("Fixed z value (mm): "))
        elif axis == 'z':
            fixed_x = float(input("Fixed x value (mm): "))
            fixed_y = float(input("Fixed y value (mm): "))
        else:
            print("Axis must be x, y, or z.")
            return  # exits main() early if axis was typed wrong

        # ---- Build the list of test values for the swept axis ----
        # (using a manual while-loop instead of range() so decimal steps work too)
        test_values = []
        current = start
        while current <= end:
            test_values.append(current)
            current += step

        # ---- Log to keep track of pass/fail at each test point ----
        results = []

        # ---- Main sweep loop ----
        for value in test_values:

            # build the full x,y,z depending on which axis is being swept
            if axis == 'x':
                x, y, z = value, fixed_y, fixed_z
            elif axis == 'y':
                x, y, z = fixed_x, value, fixed_z
            elif axis == 'z':
                x, y, z = fixed_x, fixed_y, value

            command = json.dumps({"T": 1041, "x": x, "y": y, "z": z})
            print(f"\nCandidate ({axis}={value}): {command}")

            # ---- Confirm before sending, same as before ----
            confirm = input("Send this? (y/n/q to quit sweep): ").strip().lower()
            if confirm == 'q':
                print("Sweep stopped by user.")
                break
            elif confirm != 'y':
                print("Skipped.")
                results.append((value, "skipped"))
                continue

            ser.write(command.encode() + b'\n')

            # ---- Record whether the move actually worked ----
            outcome = input("Did the arm move there safely? (y/n): ").strip().lower()
            if outcome == 'y':
                results.append((value, "pass"))
            else:
                results.append((value, "fail"))
                print("Stopping sweep due to reported failure.")
                break

        # ---- Print a summary at the end ----
        print("\n--- Sweep Results ---")
        for value, outcome in results:
            print(f"{axis}={value}: {outcome}")

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()