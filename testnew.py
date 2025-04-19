import serial

device = serial.Serial("/dev/ttyUSB2", 2000000, timeout=1)

while True:
    data = device.read(1)
    if data == b"\xaa":
        print()
    print(data, end=" ")