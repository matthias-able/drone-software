from enum import Enum
from typing import List

import Mission
import missions.emergency_landing_mission as emergency
import missions.fly_waypoints as fly_way
import missions.hop_in_place as hop_in_place
import control.controller as controller
import control.failsafe as failsafe
import copter


class MissionState(Enum):
    NEW = 1
    RUNNING = 2
    CONTINUING = 3
    CONTINUING_FROM_ERROR = 4


_height, _heading, _course, _speed = 0,0,0,0
_climb_rate = 0
_connection_lost = False

_missions: List[Mission.Mission] = []
_mission_state = MissionState.NEW  # alternatives: "running", "continuing", "continuingFromError"
_flying = True


def _append_mission(mission: Mission.Mission):
    if isinstance(mission, Mission.Mission):
        _missions.append(mission)
    else:
        raise TypeError()


def set_parameters(height=_height, heading=_heading, course=_course, speed=_speed):
    """
    To be called by missions to specify what the copter should be controlled to do. Parameters that are not given keep
    their value. If a value is given it overrides the potentially set corresponding alternative parameter (currently
    only for height and climb_rate) and sets it to None.
    All angles are given in radian.
    :param height: The target height in m.
    :param heading: The target heading (copter-orientation).
    :param course: The target course for the copter to fly.
    :param speed: The target horizontal speed to fly the copter with the target course in m/s.
    :return: None
    """
    global _height, _heading, _course, _speed, _climb_rate
    if height is not None:
        _height = height
        _climb_rate = None
    _heading = heading
    _course = course
    _speed = speed


def set_alternative_parameters(climb_rate=_climb_rate):  # , yaw_rate=_yaw_rate, lat=_lat, long=_long):
    """
    To be alternatively/additionally called by missions to specify what the copter should be controlled to do.
    Parameters that are not given keep their value. If a value is given it overrides the potentially set corresponding
    normal parameter (currently only for height and climb_rate) and sets it to None.
    :param climb_rate: The target climb rate in m/s
    :return: None
    """
    global _height, _climb_rate
    if climb_rate is not None:
        _climb_rate = climb_rate
        _height = None


def start_sub_mission(mission: Mission.Mission):
    """
    To be called to start the given mission as a sub-mission. Triggers an immediate start of that mission. When that
    mission finishes (by calling mission_finished()) continue_mission_from_submission() of the current/previous mission
    will be called, or continue_mission_from_submission_error() if an error occurres.
    :param mission: The Mission instance to be started as sub-mission. Should usually be a new instance, as multiple
    calls of this method with the same object will likely cause severe problems if the mission is not designed for that.
    :return: None
    """
    global _mission_state
    _append_mission(mission)
    _mission_state = MissionState.NEW


def mission_finished():
    """
    To be called by sub-missions when finished. Triggers returning to the previous (sub-)mission. After calling this
    no method of the current mission-object will be called anymore (except is has been started multiple times).
    :return: None
    """
    global _mission_state
    _missions.pop()
    _mission_state = MissionState.CONTINUING
    if len(_missions) == 0:
        _append_mission(emergency.EmergencyLandingMission())


def mission_error():
    global _mission_state
    # todo: log error
    mission_finished()
    _mission_state = MissionState.CONTINUING_FROM_ERROR


def flight_finished():
    """
    To be called to end the flight (shutdown the copter and end the program). The copter should be safely on the ground,
    or will otherwise crash. No method of any mission will be called by main.py anymore if called from loop_run().
    :return: None
    """
    global _flying
    _flying = False


def main_run():
    global _height, _heading, _course, _speed
    global _climb_rate
    global _connection_lost

    global _missions
    global _mission_state
    global _flying

    _append_mission(hop_in_place.HopInPlaceMission())

    print("Starting copter initialization.")
    copter.init()
    _heading = copter.get_heading()

    print("Starting controller initialization.")
    controller.init()

    print("Starting failsafe initialization.")
    failsafe.init()

    print("Finished initialization.")

    # Compare this to the following loop:
    if not (failsafe.check_connection_now() and failsafe.get_connection_up()):
        print("No connection, not starting main loop.")
        _flying = False
    else:
        copter.refresh_sensors()
        try:
            print("Starting first mission.")
            _missions[-1].start_mission()
            _mission_state = MissionState.RUNNING
            _missions[-1].loop_run()
        except Exception as ex:
            print(ex)
            _flying = False

    while _flying:
        # I think this order is best: failsafe -> sensor refresh -> mission -> controlling (like it is now)
        # If a mission turns _flying to False, control() should not be called anymore (that's why it's at the beginning
        # of the loop). The stuff to do before the first call of controller.control() is done before the loop.
        controller.control(_height, _climb_rate, _heading, _course, _speed)
        global _connection_lost
        if not _connection_lost:
            if not failsafe.get_connection_up():
                print("Connection lost, starting emergency landing.")
                start_sub_mission(emergency.EmergencyLandingMission())
                _connection_lost = True
        else:
            print("Connection lost, emergency landing.")

        copter.refresh_sensors()
        try:
            if _mission_state == MissionState.NEW:
                _missions[-1].start_as_submission()
            elif _mission_state == MissionState.CONTINUING:
                _missions[-1].continue_from_submission()
            elif _mission_state == MissionState.CONTINUING_FROM_ERROR:
                _missions[-1].continue_from_submission_error()

            _mission_state = MissionState.RUNNING

            _missions[-1].loop_run()
        except Exception as ex:  # Something could go wrong in the mission, or _missions contains an element that is no mission, ...
            print(ex)
            mission_error()

    print("Main loop ended, starting to shut down the copter.")
    copter.shutdown()
    failsafe.finish()
    print("main finished, copter has shut down. Bye!")

if __name__ == "__main__":
    import main as m
    m.main_run()
