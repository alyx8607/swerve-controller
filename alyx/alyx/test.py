import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)

while True:
    byte = ser.read(1)

    if not byte:
        continue

    if byte[0] == 0xAA:  # header found
        payload = ser.read(2)

        if len(payload) < 2:
            continue  # incomplete packet

        mode = payload[0]
        checksum = payload[1]

        # verify checksum
        if checksum == (0xAA ^ mode):
            print(f"Valid packet | Mode: {mode}")   
        else:
            print("Checksum failed")