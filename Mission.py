
import abc


class Mission(metaclass=abc.ABCMeta):
    """
    Must be extended by all missions.
    """

    @abc.abstractmethod
    def start_mission(self):
        """
        Called by main.py if this mission is the base mission for the flight. If the mission is started with this
        method, it should finish by calling flight_finished() of main.py with the copter safely on the ground.
        Should raise an error if this mission does not support to be a base mission.
        Otherwise loop_run() will be called directly afterwards, before anything is done.
        """
        pass

    @abc.abstractmethod
    def start_as_submission(self):
        """
        Called by main.py to start this mission as sub-mission as another mission. If this mission ends,
        mission_finished() of main.py should be called or flight_finished() to end the flight and the program if the
        copter is safely on the ground.
        loop_run() will be called directly afterwards, before anything is done.
        """
        pass

    @abc.abstractmethod
    def loop_run(self):
        """
        Called by main.py in every controlling loop. Should check if anything is to do and use set_parameters() of
        main.py to specify what the controller should make the copter do.
        """
        pass

    @abc.abstractmethod
    def continue_from_submission(self):
        """
        Called when a sub-mission, triggered by a call of start_submission() of main.py by this object, finished. This
        is usually (if no error occurres and this object is not started as a sub-mission again) the first method of this
        object called by main.py after the call of start_submission().
        loop_run() will be called directly afterwards, before anything is done.
        """
        pass

    @abc.abstractmethod
    def continue_from_submission_error(self):
        """
        Called when a sub-mission, triggered by a call of start_submission() of main.py by this object, raised an
        exception or isn't a mission. The error could have occurred at any point. continue_from_submission() will not be
        called if this gets called.
        loop_run() will be called directly afterwards, before anything is done.
        """
        pass
