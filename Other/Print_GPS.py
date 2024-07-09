# instructions from https://sparklers-the-makers.github.io/blog/robotics/use-neo-6m-module-with-raspberry-pi/
# NEMA sentences: https://www.gpsinformation.org/dale/nmea.htm
# best source: https://circuitdigest.com/microcontroller-projects/raspberry-pi-3-gps-module-interfacing/

import sys
import serial
import numpy as np
import pynmea2
import time

duration = 120  # Time to capture data in seconds

prev_lat = 0
prev_long = 0
last_time = 0

ser = serial.Serial("/dev/ttyAMA0", 9600, 8, 'N', 1, timeout=1)
file = open("GPS-log.txt", "a")
file.write("--- New run ---\n")
start_time = time.time()
while time.time() < start_time + duration:
    a, b = True, True
    while a == True or b == True:
        data = ser.readline()
        if sys.version_info[0] == 3:
            data = data.decode("utf-8", "ignore")
        if data[0:6] == "$GPRMC":
            #print(data)
            msg = pynmea2.parse(data)
            a = False
        if data[0:6] == "$GPVTG":
            #print(data)
            msg2 = pynmea2.parse(data)
            b = False

    time_diff = time.time() - last_time
    last_time = time.time()
    factor = np.exp(-time_diff * 1)
    lat = msg.latitude * (1-factor) + factor * prev_lat
    long = msg.longitude * (1-factor) + factor * prev_long

    dE = 111.3 * np.cos(lat * np.pi/180) * (long - prev_long) * 1000  # distance from start to copter in longitude in meter
    dN = 111.3 * (lat - prev_lat) * 1000  # distance from start to copter in latitude in meter
    speed = np.sqrt(dE ** 2 + dN ** 2) / time_diff
    if dE == 0:
        track = np.pi * (dN < 0)  # The condition evaluates to 1 if true and 0 if false
    else:
        temp = np.arctan(dN / np.absolute(dE))
        if dE < 0:
            track = temp + (3 * np.pi / 2)
        else:
            track = np.pi / 2 - temp
    track *= 180/np.pi

    prev_long = long
    prev_lat = lat

    if msg2.true_track is None:
        line = (str(time.time()-start_time) + ":\tLat°: " + str(msg.latitude) + "\tLong°: " + str(msg.longitude) + "\tSpeed: " + str(msg2.spd_over_grnd_kmph / 3.6) + "\tTrack°: 0 (not given)" + "\tCalcSpd: " + str(speed) + "\tCalcTrk°: " + str(track) + "\n")
    else:
        line = (str(time.time()-start_time) + ":\tLat°: " + str(msg.latitude) + "\tLong°: " + str(msg.longitude) + "\tSpeed: " + str(msg2.spd_over_grnd_kmph / 3.6) + "\tTrack°:" + str(msg2.true_track) + "\tCalcSpd: " + str(speed) + "\tCalcTrk°: " + str(track) + "\n")
    file.write(line)
    print(line)
file.write("--- Run finished ---\n")
file.close()
