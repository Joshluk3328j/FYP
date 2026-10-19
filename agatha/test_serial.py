import serial
import time

# Update /dev/ttyUSB0 if your port changed
PORT = '/dev/ttyUSB0'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to {PORT}")

    # Format: V[Left_Rad/s] [Right_Rad/s]
    # 2.0 rad/s is roughly 1/3 of a rotation per second
    command = "V,2.0,2.0\n" 
    print(f"Sending: {command.strip()}")
    ser.write(command.encode())
        
    # Monitor feedback for 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Nano says: {line}")
        time.sleep(0.01)

    print("Sending STOP...")
    ser.write("STOP\n".encode())
    ser.close()

except Exception as e:
    print(f"Error: {e}")