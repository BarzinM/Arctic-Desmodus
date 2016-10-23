#!/usr/bin/env python3

import serial
import struct
import threading
from time import sleep, time

checksum_const = b's'[0] + b'n'[0] + b'p'[0]
nan_value = float('Nan')


def twosCompliment(two_bytes):
    value = two_bytes[1] + (two_bytes[0] << 8)
    if (value & (1 << (16 - 1))) != 0:
        value = value - (1 << 16)
    return value


def bytesToFloat(four_bytes):
    return struct.unpack('>f', four_bytes)[0]


class D3(object):
    def __init__(self):
        self.x = nan_value
        self.y = nan_value
        self.z = nan_value
        self.time = nan_value

    def __str__(self):
        return "%.3f, %.3f, %.3f @ %.3f" % (self.x, self.y, self.z, self.time)


class PhysicalObject(object):
    def __init__(self):
        self.accel = D3()
        self.gyro = D3()
        self.gyro_rate = D3()
        self.mag = D3()
        self.euler = D3()
        self.euler_rate = D3()
        self.temp = nan_value
        self.temp_time = nan_value

    def __str__(self):
        string = "\n".join(["Accelerometer: " + str(self.accel), "Gyroscope: " + str(self.gyro), "Gyro Rate: " + str(self.gyro_rate), "Magnetometer: " +
                            str(self.mag), "Euler: " + str(self.euler), "Euler Rate: " + str(self.euler_rate), "Temperature: " + str(self.temp) + " @ %.3f" % self.temp])
        return string


class UM7(PhysicalObject):
    def __init__(self, interface='/dev/ttyUSB0', baudrate=115200):
        PhysicalObject.__init__(self)
        self.interface = serial.Serial(interface, baudrate)

    def run(self):
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run)
        self.thread.setDaemon(True)
        self.thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        serial_dev = self.interface
        checksum_const = b's'[0] + b'n'[0] + b'p'[0]
        while not self._stop.isSet():
            er = findStart(serial_dev)
            if er:
                print("Packet offset!")
            packet_type = serial_dev.read(1)[0]
            has_data, is_batch, length, failed = packetTypeParse(packet_type)
            address = serial_dev.read(1)[0]
            if has_data:
                if is_batch:
                    data_length = 4 * length
                else:
                    data_length = 4
            else:
                data_length = 0
            data = serial_dev.read(data_length)
            checksum = serial_dev.read(1)[0] << 8 | serial_dev.read(1)[0]
            computed_checksum = getChecksum(data) + packet_type + address + checksum_const
            if not checksum == computed_checksum:
                print("Checksum mismatch!")
            if has_data:
                for i in range(0, len(data), 4):
                    self.parseRegister(data[i:i + 4], address)
                    address += 1
            else:
                print("Command %s on address %d with %d batch length." % ("failed" if failed else "successful", address, length))

    def parseRegister(self, data, address):
        if address == 95:
            self.temp = bytesToFloat(data)
        elif address == 96:
            self.temp_time = bytesToFloat(data)
        elif address == 97:
            self.gyro.x = bytesToFloat(data)
        elif address == 98:
            self.gyro.y = bytesToFloat(data)
        elif address == 99:
            self.gyro.z = bytesToFloat(data)
        elif address == 100:
            self.gyro.time = bytesToFloat(data)
        elif address == 101:
            self.accel.x = bytesToFloat(data)
        elif address == 102:
            self.accel.y = bytesToFloat(data)
        elif address == 103:
            self.accel.z = bytesToFloat(data)
        elif address == 104:
            self.accel.time = bytesToFloat(data)
        elif address == 105:
            self.mag.x = bytesToFloat(data)
        elif address == 106:
            self.mag.y = bytesToFloat(data)
        elif address == 107:
            self.mag.z = bytesToFloat(data)
        elif address == 108:
            self.mag.time = bytesToFloat(data)
        elif address == 112:
            self.euler.x = twosCompliment(data[:2]) / 91.02222
            self.euler.y = twosCompliment(data[2:]) / 91.02222
        elif address == 113:
            self.euler.z = twosCompliment(data[:2]) / 91.02222
        elif address == 114:
            self.euler_rate.x = twosCompliment(data[:2]) / 16.
            self.euler_rate.y = twosCompliment(data[2:]) / 16.
        elif address == 115:
            self.euler_rate.z = twosCompliment(data[:2]) / 16.
        elif address == 116:
            self.euler.time = bytesToFloat(data)
        else:
            parseData(data, address)

    def commandZeroGyro(self):
        self.interface.write(commandPackage(173))

    def commandResetEKF(self):
        self.interface.write(commandPackage(179))

    def commandResetFactory(self):
        self.interface.write(commandPackage(172))

    def commandRawRate(self, temp, all_raw):
        self.interface.write(writePackage(2, [temp, 0, 0, all_raw]))

    def commandProcRate(self, rate):
        self.interface.write(writePackage(4, [0, 0, 0, rate]))

    def commandQuatEulerPosVelRate(self, quat, euler, pos, vel):
        self.interface.write(writePackage(5, [quat, euler, pos, vel]))

    def commandPoseRate(self, pose_rate, health_rate):
        assert health_rate < 7
        self.interface.write(writePackage(6, [pose_rate, health_rate, 0, 0]))

    def commandMagnetometerInKalman(self, include_bool):
        assert type(include_bool) == bool
        self.interface.write(writePackage(8, [0, 0, 0, int(include_bool)]))


def findStart(serial_interface):
    i = 0
    while True:
        while serial_interface.read(1) != b's':
            i += 1
        else:
            if serial_interface.read(1) == b'n' and serial_interface.read(1) == b'p':
                return i


def getChecksum(byte_array):
    computed_checksum = 0
    for byte in byte_array:
        computed_checksum += byte
    return computed_checksum


def packetTypeParse(num):
    has_data = num >> 7 & 1
    is_batch = num >> 6 & 1
    batch_length = num >> 2 & 15
    if num & 1:
        failed = True
    else:
        failed = False
    return has_data, is_batch, batch_length, failed


def parseData(data, address):
    for i in range(0, len(data), 4):
        parseRegister(data[i:i + 4], address)
        address += 1


def commandPackage(address):
    string = b'snp\x00'
    string += address.to_bytes(1, byteorder='big')
    checksum = getChecksum(string)
    string += checksum.to_bytes(2, byteorder='big')
    return string


def batchRequest(address, batch_length):
    string = b'snp'
    packet_type = 0
    assert batch_length < 17
    if batch_length > 1:
        packet_type |= batch_length << 2
    string += packet_type.to_bytes(1, byteorder='big')
    string += address.to_bytes(1, byteorder='big')
    checksum = getChecksum(string)
    string += checksum.to_bytes(2, byteorder='big')
    return string


def writePackage(address, data):
    string = b'snp'
    packet_type = 1 << 7
    assert len(data) % 4 == 0
    batch_length = len(data) // 4
    assert batch_length < 17
    if batch_length > 1:
        packet_type |= batch_length << 2
    string += packet_type.to_bytes(1, byteorder='big')
    string += address.to_bytes(1, byteorder='big')
    string += bytes(data)
    checksum = getChecksum(string)
    string += checksum.to_bytes(2, byteorder='big')
    return string


def commandZeroGyro():
    return commandPackage(173)


def commandResetEKF():
    return commandPackage(179)


def commandResetFactory():
    return commandPackage(172)


def commandRawRate(temp, all_raw):
    return writePackage(2, [temp, 0, 0, all_raw])


def commandProcRate(rate):
    return writePackage(4, [0, 0, 0, rate])


def commandQuatEulerPosVelRate(quat, euler, pos, vel):
    return writePackage(5, [quat, euler, pos, vel])


def commandPoseRate(pose_rate, health_rate):
    assert health_rate < 7
    return writePackage(6, [pose_rate, health_rate, 0, 0])


def commandMagnetometerInKalman(include_bool):
    assert type(include_bool) == bool
    return writePackage(8, [0, 0, 0, int(include_bool)])


def parseRegister(data, address):
    if address == 0:
        baudrate = data[0] >> 4
        baudlist = [9600, 14400, 19200, 38400, 57600, 115200, 128000, 153600, 230400, 256000, 460800, 921600]
        baudrate = baudlist[baudrate]
        print("Baudrate is", baudrate)

    elif address == 85:
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
    elif address == 86:
        gyro_x = twosCompliment(data[:2])
        gyro_y = twosCompliment(data[2:])
        print('gyro x, y: ', gyro_x, gyro_y)
    elif address == 87:
        gyro_z = twosCompliment(data[:2])
        print('gyro z:', gyro_z)
    elif address == 88:
        gyro_time = bytesToFloat(data)
        print('gyro time:', gyro_time)
    elif address == 89:
        accel_x = twosCompliment(data[:2])
        accel_y = twosCompliment(data[2:])
        print('accel x,y:', accel_x, accel_y)
    elif address == 90:
        accel_z = twosCompliment(data[:2])
        print('accel z:', accel_z)
    elif address == 91:
        accel_time = bytesToFloat(data)
        print('accel time:', accel_time)
    elif address == 92:
        mag_x = twosCompliment(data[:2])
        mag_y = twosCompliment(data[2:])
        print('mag x,y:', mag_x, mag_y)
    elif address == 93:
        mag_z = twosCompliment(data[:2])
        print('mag z:', mag_z)
    elif address == 94:
        mag_time = bytesToFloat(data)
        print('mag time:', mag_time)
    elif address == 95:
        temp = bytesToFloat(data)
        print('temp:', temp)
    elif address == 96:
        temp_time = bytesToFloat(data)
        print('temp time:', temp_time)
    elif address == 97:
        gyro_x = bytesToFloat(data)
        print('proc gyro x:', gyro_x)
    elif address == 98:
        gyro_y = bytesToFloat(data)
        print('proc gyro y:', gyro_y)
    elif address == 99:
        gyro_z = bytesToFloat(data)
        print('proc gyro z:', gyro_z)
    elif address == 100:
        gyro_time = bytesToFloat(data)
        print('proc gyro time:', gyro_time)
    elif address == 101:
        accel_x = bytesToFloat(data)
        print('proc accel x:', accel_x)
    elif address == 102:
        accel_y = bytesToFloat(data)
        print('proc accel y:', accel_y)
    elif address == 103:
        accel_z = bytesToFloat(data)
        print('proc accel z:', accel_z)
    elif address == 104:
        accel_time = bytesToFloat(data)
        print('proc accel time:', accel_time)
    elif address == 105:
        mag_x = bytesToFloat(data)
        print('proc mag x:', mag_x)
    elif address == 106:
        mag_y = bytesToFloat(data)
        print('proc mag y:', mag_y)
    elif address == 107:
        mag_z = bytesToFloat(data)
        print('proc mag z:', mag_z)
    elif address == 108:
        mag_time = bytesToFloat(data)
        print('proc mag time:', mag_time)
    elif address == 112:
        rpy_x = twosCompliment(data[:2]) / 91.02222
        rpy_y = twosCompliment(data[2:]) / 91.02222
        print('rpy x, y: ', rpy_x, rpy_y)
    elif address == 113:
        rpy_z = twosCompliment(data[:2]) / 91.02222
        print('rpy z:', rpy_z)
    elif address == 114:
        rate_rpy_x = twosCompliment(data[:2]) / 16.
        rate_rpy_y = twosCompliment(data[2:]) / 16.
        print('rate_rpy x, y: ', rate_rpy_x, rate_rpy_y)
    elif address == 115:
        rate_rpy_z = twosCompliment(data[:2]) / 16.
        print('rate_rpy z:', rate_rpy_z)
    elif address == 116:
        euler_time = bytesToFloat(data)
        print('euler time:', euler_time)
    elif address == 170:
        print('Version:', data)
    elif address == 172:
        print('Command to reset to factory default.')
    elif address == 173:
        print('Command to trim gyro.')
    elif address == 179:
        print('Command to reset EKF.')


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plt.ion()
    fig = plt.figure()
    plt.ylim([-2, 2])
    b = plt.bar(range(3), [0, 0, 0], width=.3)

    dev = UM7('/dev/ttyUSB0', 115200)
    dev.commandProcRate(10)
    dev.commandPoseRate(0, 0)
    dev.run()
    temp = dev.accel.x

    while True:
        new = dev.accel.x
        if abs(new - temp) > .2:
            print("BANG", time())
        temp = new
        b[0].set_height(dev.accel.x)
        b[1].set_height(dev.accel.y)
        b[2].set_height(dev.accel.z)
        fig.canvas.draw()

    dev.stop()
