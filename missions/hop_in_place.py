import Mission
import main
import copter
import control.flight_commands
import missions.emergency_landing_mission as emergency


class HopInPlaceMission(Mission.Mission):
    """
    Take off and climb to a target height, wait there and land again (via the emergency landing mission). All of that
    with a targeted horizontal speed of 0.
    """

    def __init__(self, target_height=1.5, wait_time=6, climb_rate=0.5, epsilon=0.2):
        """
        Creates a new HopInPlaceMission for a hop in place with the given properties
        :param target_height: The targeted height of the hop (in meters) (must be >0)
        :param wait_time: How long (in seconds) the copter should wait when reached the targeted height (came closer than epsilon) before starting to decent (must be >=0)
        :param climb_rate: The target climb rate (in meters per second) on the way up to the targeted height (must be >0)
        :param epsilon: The height difference (in meters) to the target height, such that if the copter is closer to the target height, the target height is considered reached (must be >0)
        """
        super().__init__()
        if target_height <= 0 or wait_time < 0 or climb_rate <= 0 or epsilon <= 0:
            raise ValueError("An argument given to the constructor of HopInPlaceMission is not appropriate")
        self._target_height = target_height
        self._wait_time = wait_time
        self._climb_rate = climb_rate
        self._epsilon = epsilon
        self._dh = 1.3 * (climb_rate**2) / (2 * 9.81)
        self._reached_time = 0


    def start_mission(self):
        """
        Called by main.py if this mission is the base mission for the flight. If the mission is started with this
        method, it should finish by calling flight_finished() of main.py with the copter safely on the ground.
        Should raise an error if this mission does not support to be a base mission.
        Otherwise loop_run() will be called directly afterwards, before anything is done.
        """
        main.set_parameters(speed=0)
        main.set_alternative_parameters(climb_rate=self._climb_rate)
        control.flight_commands.start()

    def start_as_submission(self):
        """
        Called by main.py to start this mission as sub-mission as another mission. If this mission ends,
        mission_finished() of main.py should be called or flight_finished() to end the flight and the program if the
        copter is safely on the ground.
        loop_run() will be called directly afterwards, before anything is done.
        """
        raise Exception("HopInPlaceMission is not usable as a sub-mission")

    def loop_run(self):
        """
        Called by main.py in every controlling loop. Should check if anything is to do and use set_parameters() of
        main.py to specify what the controller should make the copter do.
        """
        diff = self._target_height - copter.get_height()
        if diff <= self._dh:
            main.set_parameters(height=self._target_height)
        if self._reached_time == 0:
            if abs(diff) < self._epsilon:
                self._reached_time = copter.get_time()
                main.set_parameters(height=self._target_height)
        elif copter.get_time() - self._reached_time >= self._wait_time:
            main.start_sub_mission(emergency.EmergencyLandingMission())

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
