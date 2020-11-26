import serial, time
import numpy as np

POLYNOMINAL = np.uint8(0x9b)
CrcTable = []

ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=38400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    xonxoff=False
    )

def msg_crc_init():
    reminder = np.uint8(0)
    dividend = np.uint8(0)
    # Compute the remainder of each possible dividend.
    while True:
        reminder = dividend
        # Perform modulo-2 division, a bit at a time.
        for bit in range (8, 0, -1):
            # Try to divide the current data bit.
            if reminder & 0x80:
                reminder = np.uint8((reminder << 1) ^ POLYNOMINAL)
            else:
                reminder = np.uint8((reminder << 1))

        CrcTable.append(reminder)
        dividend+=1
        if dividend > 255:
            break

def msg_crc_calc(data, length):
    reminder = np.uint8(0)
    # Divide the message by the polynomial, a byte at a time
    for byte in range (0, length, 1):
        d = np.uint8(data[byte]) ^ np.uint8(reminder)
        reminder = np.uint8(CrcTable[d])
    #The final remainder is the CRC.
    return reminder

msg_crc_init()

packet = bytearray()
packet = [0x01, 0x01, 0x04, 0x05, 0x00, 0x05, 0xA0]
crc = msg_crc_calc(packet, packet.__len__())

for i in CrcTable:
    if CrcTable[i] == crc:
        print("CRC:", crc, "exist in CrcTable")

packet.append(crc)
print(packet)

while True:
    time.sleep(0.1)
    ser.write(packet)
    #print("Outputting transmiter buffer: " + str(ser.out_waiting))
