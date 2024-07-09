import serial

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
