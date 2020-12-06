import serial, time, random
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

MC_packet = bytearray()
MD_packet = bytearray()
MC_packet = [0x01, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x00] # MotorController packet
MD_packet = [0x01, 0x02, 0x04, 0x02, 0x02, 0x02, 0x02, 0x00] # MotorDriver packet

# Time scaler
scale_2_ms = 1e-3

while True:
    for j in range(0, 3):
        MC_packet[j+3] = random.randint(0,1)

    MD_packet[3] = random.randint(0, 28)
    MD_packet[4] = random.randint(0, 10)
    # generate RPMs in range 0-5600 rpm
    rpm = random.randint(0, 5600)
    MD_packet[5] = np.uint8(rpm >> 8)
    MD_packet[6] = np.uint8(rpm)

    # Calculate CRC
    _MC_crc = msg_crc_calc(MC_packet, 7)
    _MD_crc = msg_crc_calc(MD_packet, 7)

    MC_packet[7] = _MC_crc
    MD_packet[7] = _MD_crc

    # Send frame
    ser.write(MC_packet)
    time.sleep(10 * scale_2_ms) # 10 ms
    ser.write(MD_packet)
    time.sleep(10 * scale_2_ms) # 10 ms
