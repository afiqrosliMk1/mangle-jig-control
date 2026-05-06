import serial

#===SETTINGS===
COM_PORT = "COM14"
BAUD_RATE = 9600
message = ""

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {COM_PORT}")
except Exception as e:
    print("Failed to connect:" , e)
    ser = None

if 
    ser.write(message.encode())
