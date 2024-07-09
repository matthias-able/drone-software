import numpy as np

import copter
from control import flight_commands

_last_time = 0
_last_heading = 0
#_last_gps = (0, 0)
_last_horiz_speed = (0, 0)
_last_climb_rate = 0

_err_sum_height, _err_sum_heading, _err_sum_horiz = 0,0,0
_err_sum_climb_rate = 0

_height_scale_factor = 10  # to scale the values to [0, 100]
_proportional_factor_height = 1
_integral_factor_height = 0.08
_differential_factor_height = 0.8      # 1 before, reacts very strongly

_heading_scale_factor = 0.1/(np.pi/2)  # to scale the values to [0, 100]
_proportional_factor_heading = 1
_integral_factor_heading = 0.1
_differential_factor_heading = 0.1

_horiz_scale_factor = 100/5
_proportional_factor_horiz = 1
_integral_factor_horiz = 0.1
_differential_factor_horiz = 0.1

_climb_rate_scale_factor = 10  # to scale the values to [0, 100] # todo: initial values
_proportional_factor_climb_rate = 1
_integral_factor_climb_rate = 0.1
_differential_factor_climb_rate = 0.1

def init():
    """
    Initializes important values (position information, assuming there is no motion in the beginning) in order to
    calculate how they change later. Must be called after the copter module has been initialized.
    :return: None
    """
    global _last_time, _last_heading #, _last_gps
    _last_time = copter.get_time()
    _last_heading = copter.get_heading() # At the moment this init function is just for the heading, the rest should be 0 anyways
    #_last_gps = copter.get_coordinates_relative()

def control(target_height, target_climb_rate, target_heading, target_course, target_speed):
    """
    check params in main, calculates the signals, sends them via flight_commands to the Arduino.
    Controls height if target_height is not None, if otherwise target_climb_rate is not None, controls climb rate.
    Either target_height or target_climb_rate should not be None.

    """
    global _last_time, _last_heading, _last_horiz_speed #, _last_gps
    global _last_climb_rate
    global _err_sum_height, _err_sum_heading, _err_sum_horiz
    global _err_sum_climb_rate
    current_time = copter.get_time()
    time_diff = current_time - _last_time
    if time_diff > 0:
        current_height = copter.get_height()
        current_climb_rate = copter.get_climb_rate()
        if target_height is not None:
            height_error = target_height - current_height
            result_throttle_pd = height_error * _proportional_factor_height\
                                 - current_climb_rate * _differential_factor_height
            result_throttle = result_throttle_pd + _err_sum_height
            result_throttle = 50 + result_throttle * _height_scale_factor
            if result_throttle >= 100:
                result_throttle = 100
                _err_sum_height = (50 / _height_scale_factor) - result_throttle_pd
            elif result_throttle <= 0:
                result_throttle = 0
                _err_sum_height = (-50 / _height_scale_factor) - result_throttle_pd
            print("Target height mode:")
            print("P: " + str(height_error * _proportional_factor_height * _height_scale_factor)+ "\tD: " + str(-current_climb_rate*_differential_factor_height * _height_scale_factor)+ "\tI: " + str(_err_sum_height * _height_scale_factor))
            _err_sum_height += height_error * _integral_factor_height
        elif target_climb_rate is not None:
            vert_acc = (current_climb_rate - _last_climb_rate) / time_diff
            climb_rate_error = target_climb_rate - current_climb_rate
            result_throttle_pd = climb_rate_error * _proportional_factor_climb_rate\
                                 - vert_acc * _differential_factor_climb_rate
            result_throttle = result_throttle_pd + _err_sum_climb_rate
            result_throttle = 50 + result_throttle * _climb_rate_scale_factor
            if result_throttle >= 100:
                result_throttle = 100
                _err_sum_climb_rate = (50 / _climb_rate_scale_factor) - result_throttle_pd
            elif result_throttle <= 0:
                result_throttle = 0
                _err_sum_climb_rate = (-50 / _climb_rate_scale_factor) - result_throttle_pd
            print("Target climb rate mode:")
            print("P: " + str(climb_rate_error * _proportional_factor_climb_rate * _climb_rate_scale_factor)+ "\tD: " + str(-vert_acc*_differential_factor_climb_rate * _climb_rate_scale_factor)+ "\tI: " + str(_err_sum_climb_rate * _climb_rate_scale_factor))
            _err_sum_climb_rate += climb_rate_error * _integral_factor_height


        current_heading = copter.get_heading()
        heading_rate = (current_heading - _last_heading)
        if np.absolute(heading_rate) > np.pi:
            heading_rate -= np.sign(heading_rate) * 2 * np.pi
        heading_rate /= time_diff
        heading_error = target_heading - current_heading
        if np.absolute(heading_error) > np.pi:
            heading_error -= np.sign(heading_error) * 2 * np.pi
        result_heading_pd = heading_error * _proportional_factor_heading\
                            - heading_rate * _differential_factor_heading
        result_heading = result_heading_pd + _err_sum_heading
        result_heading = 50 + result_heading * _heading_scale_factor
        if result_heading >= 100:
            result_heading = 100
            _err_sum_heading = (50 / _height_scale_factor) - result_heading_pd
        elif result_heading <= 0:
            result_heading = 0
            _err_sum_heading = (-50 / _height_scale_factor) - result_heading_pd
        _err_sum_heading += heading_error * _integral_factor_heading

        target_horiz_speed = np.multiply( (np.cos(target_course), np.sin(target_course)) , target_speed)
        current_horiz_speed = np.multiply((np.cos(copter.get_track()), np.sin(copter.get_track())), copter.get_speed())
        horiz_speed_diff = np.subtract(current_horiz_speed, _last_horiz_speed)
        horiz_acc = np.divide(horiz_speed_diff, time_diff)
        horiz_speed_error = np.subtract(current_horiz_speed, target_horiz_speed)
        result_horiz_speed = horiz_speed_error * _proportional_factor_horiz\
                            - horiz_acc * _differential_factor_horiz\
                            + _err_sum_horiz
        result_horiz_speed = np.multiply(result_horiz_speed, _horiz_scale_factor)
        horiz_speed_mag = np.sqrt(np.sum(np.square(result_horiz_speed)))
        if horiz_speed_mag >= 50:
            result_horiz_speed = np.multiply(result_horiz_speed, 50 / horiz_speed_mag)
        else:
            _err_sum_horiz = np.add(_err_sum_horiz, np.multiply(result_horiz_speed, _integral_factor_horiz))

        result_pitch = result_horiz_speed[0] * np.cos(current_heading) + result_horiz_speed[1] * np.sin(current_heading)
        result_roll = result_horiz_speed[0] * np.sin(current_heading) + result_horiz_speed[1] * np.cos(current_heading)
        result_pitch += 50
        result_roll += 50

        print('roll: ' + str(result_roll)+ ' pitch: ' + str(result_pitch)+' throttle: ' + str(result_throttle) + ' yaw: ' + str(result_heading))
        flight_commands.throttle(result_throttle)
#        result_heading = 50
        flight_commands.yaw(result_heading)
        flight_commands.pitch(result_pitch)
        flight_commands.roll(result_roll)

        _last_horiz_speed = current_horiz_speed
        _last_heading = current_heading
        _last_climb_rate = current_climb_rate
        #_last_gps = copter.get_coordinates_relative()
        _last_time = current_time
