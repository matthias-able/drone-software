# instructions from https://sparklers-the-makers.github.io/blog/robotics/use-neo-6m-module-with-raspberry-pi/
# NEMA sentences: https://www.gpsinformation.org/dale/nmea.htm
# best source: https://circuitdigest.com/microcontroller-projects/raspberry-pi-3-gps-module-interfacing/

import sys
import serial
import numpy as np
import pynmea2
import threading

_ser = None
_lat, _long, _speed, _track = 0, 0, 0, 0
"""
latitude, longitude, speed over ground as tuple, true track (traveling angle from north) (radian, radian, m/s, radian)
"""

_gps_loop_running = False
_value_lock, _flag_lock = None, None
_gps_thread = None

def init():
    """
    Initializes the connection to the gps and starts an infinite loop calling read_line() in a new thread.
    :return: None
    """
    global _ser, _lat, _long, _speed, _track, _gps_loop_running, _value_lock, _flag_lock, _gps_thread
    if _gps_loop_running:
        print("gps.init() called more than once.")
        return
    _ser = serial.Serial("/dev/ttyAMA0", 9600, 8, 'N', 1, timeout = 1)
    _value_lock = threading.Lock()
    _flag_lock = threading.Lock()
    _gps_loop_running = True
    _gps_thread = threading.Thread(target=_gps_loop)
    _gps_thread.start()

def finish():
    """
    Ends the loop and the thread started in init()
    :return:
    """
    global _gps_loop_running, _flag_lock
    if not _gps_loop_running:
        print("gps.finish() called but the gps loop doesn't run.")
        return
    _flag_lock.acquire()
    _gps_loop_running = False
    _flag_lock.release()
    _gps_thread.join()

def read_line():
    """
    Must not be called before init() is called. Read a line of the gps output and if there are relevant values in it,
    saves them in the modules variables. To be called in the separate gps loop thread created in init()
    """
    global _lat, _long, _speed, _track, _value_lock
    try:
        data = _ser.readline()
        if sys.version_info[0] == 3: # Check if using python3
            data = data.decode("utf-8","ignore")
        if data[0:6] == "$GPRMC":
            msg = pynmea2.parse(data)
            _value_lock.acquire()
            _lat = msg.latitude * np.pi/180
            _long = msg.longitude * np.pi/180
            _value_lock.release()
        if data[0:6] == "$GPVTG":
            msg = pynmea2.parse(data)
            _value_lock.acquire()
            if msg.true_track is not None:
                _speed = msg.spd_over_grnd_kmph / 3.6
                _track = msg.true_track * np.pi/180
            else:
                _speed = 0
            _value_lock.release()
    except Exception as ex:
        print("Exception reading or interpreting gps data")
        print(ex)

def _gps_loop():
    _flag_lock.acquire()
    while _gps_loop_running:
        _flag_lock.release()
        read_line()
        _flag_lock.acquire()

def get_values():
    """
    Returns the latest gps values. To be called by the main thread.
    :return: Tuple (latitude, longitude, speed over ground as tuple, true track (traveling angle from north)) (radian, radian, m/s, radian)
    """
    _value_lock.acquire()
    values = _lat, _long, _speed, _track
    _value_lock.release()
    print("get values: " + str(values))
    return values

