import serial

ser = serial.Serial('COM5', 9600, timeout=1)
print("COM5 pe sun raha hoon...")

while True:
    data = ser.readline()
    if data:
        print(data)
