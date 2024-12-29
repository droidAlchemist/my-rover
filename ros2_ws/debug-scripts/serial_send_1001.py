import serial
import json  

# Initialize serial communication with the UGV
serial_port = '/dev/ttyAMA0'
ser = serial.Serial(serial_port, 115200, timeout=1)

def main():
    ctrl_data = json.dumps({"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}) + "\n"                
    ser.write(ctrl_data.encode())        
    ser.close()


if __name__ == "__main__":
    main()