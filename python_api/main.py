import serial
import struct


def findStart(serial_interface):
    i = 0
    while True:
        while serial_interface.read(1) != b's':
            i += 1
            # print(c,end='')
        else:
            if serial_interface.read(1) == b'n' and serial_interface.read(1) == b'p':
                return i


def getChecksum(byte_array):
    computed_checksum = 0
    for byte in byte_array:
        # print(byte)
        computed_checksum += byte
    return computed_checksum


def packetTypeParse(byte):
    has_data = byte[0] >> 7 & 1
    is_batch = byte[0] >> 6 & 1
    batch_length = byte[0] >> 2 & 15
    return has_data, is_batch, batch_length


def parseData(data, address):
    for i in range(0, len(data), 4):
        parseRegister(data[i:i + 4], address)
        address += 1


def twosCompliment(two_bytes):
    value = two_bytes[1]+(two_bytes[0]<<8)
    if (value & (1 << (16 - 1))) != 0:
        value = value - (1 << 16)
    return value

def bytesToFloat(four_bytes):
    return struct.unpack('>f', four_bytes)[0]

def parseRegister(data, address):
    if address == 85:
        if data[2] & 1:
            print('Overflow warning!')
        if data[3] & 32:
            print('Magnetometer norm error!')
        if data[3] & 16:
            print('Accelerometer norm error!')
        if data[3] & 8:
            print('Accelerometer failed to initialize on startup!')
        if data[3] & 4:
            print('Gyro failed to initialize on startup!')
        if data[3] & 2:
            print('Magnetometer failed to initialize on startup!')
        if data[3] & 1:
            print('GPS hasn\'t send a packet for more than two seconds!')
    if address == 86:
        gyro_x = twosCompliment(data[:2])
        gyro_y = twosCompliment(data[2:])
        print('gyro x, y: ',gyro_x,gyro_y)
    if address == 87:
        gyro_z = twosCompliment(data[:2])
        print('gyro z:',gyro_z)
    if address == 88:
        gyro_time = bytesToFloat(data)
        print('gyro time:', gyro_time)
    if address == 89:
        accel_x = twosCompliment(data[:2])
        accel_y = twosCompliment(data[2:])
        print('accel x,y:',accel_x,accel_y)
    if address ==  90:
        accel_z = twosCompliment(data[:2])
        print('accel z:',accel_z)
    if address == 91:
        accel_time = bytesToFloat(data)
        print('accel time:', accel_time)
    if address == 92:
        mag_x = twosCompliment(data[:2])
        mag_y = twosCompliment(data[2:])
        print('mag x,y:',mag_x,mag_y)
    if address ==  93:
        mag_z = twosCompliment(data[:2])
        print('mag z:',mag_z)
    if address == 94:
        mag_time = bytesToFloat(data)
        print('mag time:', mag_time)
    if address == 95:
        temp = bytesToFloat(data)
        print('temp:',temp)
    if address == 96:
        temp_time = bytesToFloat(data)
        print('temp time:', temp_time)
    if address == 97:
        gyro_x = bytesToFloat(data)
        print('proc gyro x:',gyro_x)
    if address == 98:
        gyro_y = bytesToFloat(data)
        print('proc gyro y:',gyro_y)
    if address == 99:
        gyro_z = bytesToFloat(data)
        print('proc gyro z:',gyro_z)
    if address == 100:
        gyro_time = bytesToFloat(data)
        print('proc gyro time:', gyro_time)
    if address == 101:
        accel_x = bytesToFloat(data)
        print('proc accel x:',accel_x)
    if address == 102:
        accel_y = bytesToFloat(data)
        print('proc accel y:',accel_y)
    if address ==  103:
        accel_z = bytesToFloat(data)
        print('proc accel z:',accel_z)
    if address == 104:
        accel_time = bytesToFloat(data)
        print('proc accel time:', accel_time)
    if address == 105:
        mag_x = bytesToFloat(data)
        print('proc mag x:',mag_x)
    if address == 106:
        mag_y = bytesToFloat(data)
        print('proc mag y:',mag_y)
    if address ==  107:
        mag_z = bytesToFloat(data)
        print('proc mag z:',mag_z)
    if address == 108:
        mag_time = bytesToFloat(data)
        print('proc mag time:', mag_time)
    if address == 112:
        rpy_x = twosCompliment(data[:2])/91.02222
        rpy_y = twosCompliment(data[2:])/91.02222
        print('rpy x, y: ',rpy_x,rpy_y)
    if address == 113:
        rpy_z = twosCompliment(data[:2])/91.02222
        print('rpy z:',rpy_z)
    if address == 114:
        rate_rpy_x = twosCompliment(data[:2])/16.
        rate_rpy_y = twosCompliment(data[2:])/16.
        print('rate_rpy x, y: ',rate_rpy_x,rate_rpy_y)
    if address == 115:
        rate_rpy_z = twosCompliment(data[:2])/16.
        print('rate_rpy z:',rate_rpy_z)
    if address == 104:
        euler_time = bytesToFloat(data)
        print('euler time:', euler_time)

baudrate = 115200
dev = serial.Serial('/dev/ttyUSB0', baudrate)
checksum_const = b's'[0] + b'n'[0] + b'p'[0]

for i in range(20):
    er = findStart(dev)

    packet_type = dev.read(1)
    has_data, is_batch, length = packetTypeParse(packet_type)
    address = dev.read(1)
    if is_batch:
        data_length = 4 * length
    else:
        data_length = 4
    data = dev.read(data_length)
    checksum = dev.read(1)[0] << 8 | dev.read(1)[0]
    computed_checksum = getChecksum(data) + packet_type[0] + address[0] + checksum_const
    print(packet_type, has_data != 0, is_batch != 0, length, address[0], checksum == computed_checksum, 'offset:', er)
    parseData(data, address[0])
