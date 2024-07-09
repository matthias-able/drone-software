import sensors.gps as gps
import sensors.Adafruit_BMP.BMP085 as BMP085
#import sensors.py-qmc5883l as py-qmc5883l
from sensors.pyqmc5883l import py_qmc5883l
from control import flight_commands
#from picamera import PiCamera
#from PIL import Image
 
import RPi.GPIO as GPIO
import numpy as np
import time

_start_lat, _start_lon, _start_time, _ground = 0, 0, 0, 0
_lat, _lon = 0, 0
_coordinates_relative = 0, 0
_distance = 0
_heading = 0
_speed = 0
_track = 0
_height = 0
_last_height = 0
_climb_rate = 0
_time = 0  # todo check unit correction factors in control
_last_time = 0
"""
The time of the last sensor reading (via refresh_sensors() or init()) in seconds.
now: float in seconds
"""


def init():
    """
    Initialize sensors and store starting point.
    :return: None
    """
    global compass, bmp, camera

    print("Starting initialization of the connection to the Arduino.")
    # enable serial connectinon to raspberry
    flight_commands.init()

    print("Starting the initialization of the gps")
#    gps.init()

    # compass setup
    # compass i2c port 1: 3 SDA, 5 SDC
    # change /etc/profile
    compass = py_qmc5883l.QMC5883L()
    # compass.declination = 3.5  # Erlangen (degree=3, min=28)
    compass.declination = 5.6 # compass.setDeclination(degree = 5, min = 36) # Gaziantep Turkey

    # GPS setup
    # gps over UART: /dev/ttyAMA0 pin 8 TX, 10 RX
    # set up UART: change /boot/config.txt and  boot/cmdline.txt, Activating ttyAMAO
    # port = "/dev/ttyAMA0" maybe give this into gps.mwasure() necessary

    # pressure sensor setup
    # pressure i2c port 1: pin 3 SDA, pin 5 SCL
    # change: /etc/modules, then blacklist
    bmp = BMP085.BMP085()#0x77, BMP085_ULTRAHIGHRES) # ULTRAHIRES Mode #todo: change adress

    # camera setup
    #camera = PiCamera()

    # motor signal setup for water release system
    #    GPIO.setmode(GPIO.BOARD) # Declare the GPIO settings
    # set up GPIO pins
    #    GPIO.setup(15, GPIO.OUT) # Connected to PWMA
    #    GPIO.setup(16, GPIO.OUT) # Connected to AIN2
    #    GPIO.setup(18, GPIO.OUT) # Connected to AIN1
    #    GPIO.setup(13, GPIO.OUT) # Connected to STBY

    global _start_lat, _start_lon, _speed, _track, _start_time, _last_time, _time, _ground, _last_height

    print("Starting reading first sensor values.")

    _start_lat = 0
 #   wait_start_time = time.time()
 #   while _start_lat == 0:
 #       _start_lat, _start_lon, _speed, _track = gps.get_values()  # store starting position
 #       if time.time() - wait_start_time > 10:
 #           raise TimeoutError("No GPS signal for 10 seconds")

    _ground = 0
    for i in range(50):
        _ground += bmp.read_altitude()
    _ground /= 50
    _last_height = _ground

    _start_time = time.time()  # time in seconds since the epoch as a floating point number
    _time = 0

    refresh_sensors() # Make sure all values are read and can be obtained by the getter functions from now on


def refresh_sensors():
    """
    Reads all sensors and stores the results that can then be obtained by the getter functions. Also stores the time
    passed since the last call and the current time.
    :return: None
    """
    global _distance, _height, _last_height, _climb_rate, _heading, _coordinates_relative, _lat, _lon, _speed, _track, _time, _last_time
    _last_time = _time
    _time = time.time() - _start_time

    _lat, _lon, _speed, _track = 0,0,0,0 #gps.get_values()
    dE = np.pi/180 * 111.3 * np.cos(_lat) * (_lon - _start_lon) * 1000  # distance from start to copter in longitude in meter
    dN = np.pi/180 * 111.3 * (_lat - _start_lat) * 1000  # distance from start to copter in latitude in meter
    _distance = np.sqrt(dE ** 2 + dN ** 2)
    _coordinates_relative = dE, dN

    _heading = compass.get_bearing() * np.pi/180

    current_height = bmp.read_altitude()
    cur_climb_rate = (current_height - _last_height) / (_time - _last_time)

    climb_rate_avg_factor = np.exp(1 * (_last_time - _time)) # Becomes 1/e after 1 second
    _climb_rate = (1-climb_rate_avg_factor) * cur_climb_rate + climb_rate_avg_factor * _climb_rate

    height_avg_factor = np.exp(1 * (_last_time - _time)) # Becomes 1/e after 1 second
    _height = (1-height_avg_factor) * (current_height - _ground) + height_avg_factor * _height

    _last_height = current_height

    print("")
    print("Height (m)" + str(_height))
    print("Climbrate (m/s)" + str(_climb_rate))
    print("Heading (rad)" + str(_heading))
    print("Speed (m/s)" + str(_speed))
    flight_commands.print_status()

def shutdown():
    """
    Cutoff all motors and finish everything
    :return: None
    """
    flight_commands.stop_all()
    gps.finish()


def get_time():
    """
    :return: The time of the last sensor reading (via refresh_sensors() or init()) in seconds.
    """
    return _time


def get_coordinates():
    """
    :return: lat, long both in radian
    :rtype: (float, float)
    """
    return _lat, _lon


def get_coordinates_relative():
    """
    :return: east, north in meters from the starting point
    :rtype: (float, float)
    """
    return _coordinates_relative


def get_distance():
    """
    :return: Distance from starting point in meters
    :rtype: float
    """
    return _distance


def get_start_coord():
    """
    :return: _start_lat, _start_lon
    :rtype: float, float
    """
    return _start_lat, _start_lon


def get_speed():
    """
    :return: Ground speed in m/s
    :rtype: float
    """
    return _speed


def get_track():
    """
    :return: true track (traveling angle in radian from north)
    :rtype: float
    """
    return _track


def get_heading():
    """
    :return: Heading in radians within [0, 2pi) from north.
    :rtype: float
    """
    return _heading


def get_height():
    """
    :return: Height above the ground (after averaging) in meters
    :rtype: float
    """
    return _height

def get_climb_rate():
    """
    :return: Climb rate after averaging in m/s
    :rtype: float
    """
    return _climb_rate

def get_picture(number):
    """
    Take jpg picture and store on desktop as "imagenumber.jpg"
    :param number: todo
    :return: None
    """
    camera.take_picture(number) #todo: should be maybe camera.capture()

def run_DC_motor(up):

    if up == True:
        GPIO.output(18, GPIO.HIGH)
        pwm = GPIO.PWM(15, 25)  # 25 Hz
        pwm.start(65) # 65% power
        time.sleep(2)
        pwm.stop()
    else:
        GPIO.output(18, GPIO.LOW)
        pwm = GPIO.PWM(15, 25)  # 25 Hz
        pwm.start(65) # 65% power
        time.sleep(2)
        pwm.stop()


def print_info():

    print("time in sec  " + str(get_time()))
    print("coordinates in radian  " + str(get_coordinates()))
    print("coordinates_relative in meter  " + str(get_coordinates_relative()))
    print("distance from start in meter  " + str(get_distance()))
    print("start coordinates in radian  " + str(get_start_coord()))
    print("speed in m/s  " + str(get_speed()))
    print("track in radian  " + str(get_track()))
    print("heading in radian  " + str(get_heading()))
    print("height in m above ground  " + str(get_height()))
    print("Time it took to init(), print all, refresh sensors  " + str(time.time()-_start_time))

    #z0 = time.time()
    #im = Image.open('/home/pi/Pictures/firstselfie.jpg')
    #print("Time it takes to load the image  "+str(time.time()-z0))

