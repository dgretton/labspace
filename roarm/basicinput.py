import serial
import argparse
import threading
import json

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')
    args = parser.parse_args()
    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)
    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()
    try:
        while True:
            coords = input("Enter x,y,z in mm: ")
            x, y, z = [float(v) for v in coords.split(",")]
            command = json.dumps({"T": 1041, "x": x, "y": y, "z": z})

            print(f"Candidate: {command}")
            if input("Send this? (y/n): ").strip().lower() != 'y':
                continue

            ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()