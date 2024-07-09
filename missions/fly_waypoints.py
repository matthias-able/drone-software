import copter
import control.flight_commands
import missions.emergency_landing_mission
import main
import numpy as np
import Mission


def calculate_course(start, end):
    """
    Calculates the course to fly in radian from north clockwise
    :return: radian
    :rtype: (float, float)
    """
    dE = 111.3 * np.cos(start[0]) * (end[1] - start[1]) * 1000  # distance from start to copter in longitude in meter
    dN = 111.3 * (end[0] - start[0]) * 1000  # distance from start to copter in latitude in meter

    if dE == 0:
        return np.pi * (dN < 0)  # The condition evaluates to 1 if true and 0 if false

    result = np.arctan(dN / np.absolute(dE))

    if dE < 0:
        return result + (3 * np.pi / 2)

    return np.pi / 2 - result


def calculate_dist_to_waypoint(waypoint):
    start = copter.get_coordinates()
    dE = 111.3 * np.cos(start[0]) * (
                waypoint[1] - start[1]) * 1000  # distance from waypoint to copter in longitude in meter
    dN = 111.3 * (waypoint[0] - start[0]) * 1000  # distance from waypoint to copter in latitude in meter
    return np.sqrt(dE ** 2 + dN ** 2)


class FlyWaypointsMission(Mission.Mission):

    def __init__(self, waypointfile='waypoints_test.txt'):
        """
        While creating this object, the current waypointfile musst be given as string
        :return: None
        """
        self._waypoints = np.loadtxt(waypointfile) * np.pi / 180  # load waypoints to array in radians
        self._step = -1
        self._position = [0,0]
        self._is_base_mission = False


    def _start(self):
        self._position = copter.get_coordinates()
        self._waypoints = np.vstack((self._waypoints, self._position))  # add starting position to waypoints array


    def start_mission(self):
        """
        Called by main.py if this mission is the base mission for the flight. If the mission is started with this
        method, it should finish by calling flight_finished() of main.py with the copter safely on the ground.
        Should raise an error if this mission does not support to be a base mission.
        Otherwise loop_run() will be called directly afterwards, before anything is done.
        """
        self._is_base_mission = True
        control.flight_commands.start()
        self._start()


    def start_as_submission(self):
        """
        Called by main.py to start this mission as sub-mission as another mission. If this mission ends,
        mission_finished() of main.py should be called or flight_finished() to end the flight and the program if the
        copter is safely on the ground.
        loop_run() will be called directly afterwards, before anything is done.
        """
        self._is_base_mission = False
        self._start()
        

    def loop_run(self):
        """
        Called by main.py in every controlling loop. Should check if anything is to do and use set_parameters() of
        main.py to specify what the controller should make the copter do.
        """
        self._position = copter.get_coordinates()
        
        if self._step == -1:
            main.set_parameters(height=5, heading=copter.get_heading(), speed=0)
            if copter.get_height() >= 2.5:
                self._step += 1
            else:
                main.set_alternative_parameters(climb_rate=0.5)

        elif self._step < np.size(self._waypoints, 0):
            course_1 = calculate_course(self._position, self._waypoints[self._step,:])
            main.set_parameters(height=5, course=course_1, speed=1) #todo: experiment whats highest possible speed value :)
            if calculate_dist_to_waypoint(self._waypoints[self._step,:]) < 2.5:  #todo: increase to 5 ?
                self._step += 1

        elif self._step >= np.size(self._waypoints, 0):
            if self._is_base_mission:
                main.start_sub_mission(missions.emergency_landing_mission.EmergencyLandingMission())
            else:
                main.set_parameters(speed=0)


    def change_step(new_step):
        pass


    def add_waypoint(coordinates):
        pass
        #todo: Implement as list to add waypoints in the middle? Maybe necessary for mission 2


    def continue_from_submission(self):
        """
        Called when a sub-mission, triggered by a call of start_submission() of main.py by this object, finished. This
        is usually (if no error occurres and this object is not started as a sub-mission again) the first method of this
        object called by main.py after the call of start_submission().
        loop_run() will be called directly afterwards, before anything is done.
        """
        pass


    def continue_from_submission_error(self):
        """
        Called when a sub-mission, triggered by a call of start_submission() of main.py by this object, raised an
        exception or isn't a mission. The error could have occurred at any point. continue_from_submission() will not be
        called if this gets called.
        loop_run() will be called directly afterwards, before anything is done.
        """
        pass
