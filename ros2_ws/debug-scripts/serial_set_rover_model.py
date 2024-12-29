import serial
import json  

# Initialize serial communication with the UGV
serial_port = '/dev/ttyAMA0'
ser = serial.Serial(serial_port, 115200, timeout=1)

def main():
    ctrl_data = json.dumps({
            "T":900,
            "main": 1,
            "module": 0
        }) + "\n"     
    ser.write(ctrl_data.encode())     
    ser.close()

if __name__ == "__main__":
    main()