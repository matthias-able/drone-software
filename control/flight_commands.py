# https://pythonhosted.org/RPIO/pwm_py.html#subcycles
# https://www.teachmemicro.com/raspberry-pi-pwm-servo-tutorial/

import serial
from time import sleep

def init():
    """
    starts serial connection, 4 possible ports until now :)
    :return: None
    """
    global s
    try:
        s = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
    except OSError as e:
        try:
            s = serial.Serial('/dev/ttyACM1', 57600, timeout=1)
        except OSError as e:
            try:
                s = serial.Serial('/dev/ttyACM2', 57600, timeout=1)
            except OSError as e:
                s = serial.Serial('/dev/ttyACM3', 57600, timeout=1)

    for i in range(50):
        a = s.readline() #.decode('utf-8')
        if a == b'Ready!\r\n':
            break
        elif a == b'Arduino: Checking EEPROM signature\r\n':
            print('Arduino: Checking EEPROM signature and calibrating')
        elif a == b'Arduino: Gyro not recognized!\r\n':
            raise TimeoutError("Arduino: Gyro not recognized!")

        if i >= 49:
            raise TimeoutError("Arduino didn't send line \"Ready!\" for 50 seconds")

    neutral() # send neutral comand
    s.write(b"\xF9")  # send that RasPi is ready too

    print('Arduino sent "Ready!", waiting for "Starting".')

    for i in range(50):
        if s.readline() == b'Starting\r\n':
            break
        if i >= 49:
            raise TimeoutError("Arduino didn't send line \"Starting\" for 50 seconds. Maybe it doesn't get signals from the RC-receiver?")

    neutral() # send neutral again # necessary?

def print_status():
    loop_time_exceeded = False
    escs = "No values"
    battery = "No value"
    loop_time = "No value"
    while s.in_waiting > 0:
        line = s.readline()
        if line == b'Arduino: status\r\n':
            escs = [int.from_bytes(s.read(1), "big", signed=False)*0.4 for i in range(4)]
            battery = int.from_bytes(s.read(1), "big", signed=False)*0.06
            loop_time = int.from_bytes(s.read(1), "big", signed=False)*16
        elif line == b'Arduino: loop time exceeds 4050us\r\n':
            loop_time_exceeded = True
    if loop_time_exceeded:
        print('Arduino: Loop time exceeded! ')
    print('Arduino status ESCs (%): ' + str(escs))
    print('Arduino status battery (V): ' + str(battery))    
    print('Arduino status loop time (us): ' + str(loop_time))


def start():
    """
    sends starting byte to Arduino
    :return: None
    """
    s.write(b"\xFA")


def stop_all():
    """
    sends stopping byte to Arduino
    :return: None
    """
    neutral() #todo: not necessary?
    s.write(b"\xFB")


def neutral():
    """
    neutral: roll, pitch, yaw; zero throttle
    :params: None
    :return: None
    """
    s.write(b"\xFC")
    s.write((100).to_bytes(1, "big"))
    s.write(b"\xFD")
    s.write((100).to_bytes(1, "big"))
    s.write(b"\xFE")
    s.write((0).to_bytes(1, "big"))
    s.write(b"\xFF")
    s.write((100).to_bytes(1, "big"))


def roll(percent):
    """
    Set roll signal on GPIO 17: 50 % equal to neutral position with 1500 µs
    subcycle_time_us=20000
    :params: Percent
    :return: None
    """
    # 0% = 1000ms = 5% of 20 000ms, 100% = 2000ms = 10% of 20 000ms
    percent = percent * 2
#    print('Flight commands roll: ' + str(percent))
    s.write(b"\xFC")
    s.write(int(percent).to_bytes(1, "big"))


def pitch(percent):
    """
    Set pitch signal on GPIO 17: 50 % equal to neutral position with 1500 µs
    subcycle_time_us=20000
    :params: Percent
    :return: None
    """
    percent = percent * 2
    s.write(b"\xFD")
    s.write(int(percent).to_bytes(1, "big"))


def throttle(percent):
    """
    Set throttle signal on GPIO 17: 50 % equal to neutral position with 1500 µs
    subcycle_time_us=20000
    :params: Percent
    :return: None
    """
    percent = percent * 2
    s.write(b"\xFE")
    s.write(int(percent).to_bytes(1, "big"))

def yaw(percent):
    """
    Set Signal on GPIO 17: 50 % equal to neutral position with 1500 µs
    subcycle_time_us=20000
    :params: Percent
    :return: None
    """
    percent = percent * 2
    s.write(b"\xFF")
    s.write(int(percent).to_bytes(1, "big"))


