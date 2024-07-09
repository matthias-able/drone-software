import Mission
import main
import copter

import time

class EmergencyLandingMission(Mission.Mission):
    """
    Lands the copter roughly at the current position. Can be used als last alternative if the missions fail.
    Assumes that all main systems are working (program, Arduino(-communication), pressure/height-measurement, GPS)
    So basically its only use case is if there is an error/exception thrown in the running mission and with all
    other software and hardware (and environment (e.g. unchanged pressure for height measurement, same ground level
    as starting point, ...)) working flawlessly...
    """

    def __init__(self):
        super().__init__()
        self._low_time = 0

    def start_mission(self):
        main.set_parameters(speed=0)

    def start_as_submission(self):
        self.start_mission()

    def loop_run(self):
        height = copter.get_height()
        if height <= 0.5:
            if self._low_time == 0:
                self._low_time = time.time()
            elif time.time() - self._low_time >= 4:
                main.flight_finished()
        elif height <= 10:
            main.set_alternative_parameters(climb_rate=-0.2)  # decent with 20cm/s
        elif height <= 20:
            main.set_alternative_parameters(climb_rate=-0.3)  # decent with 30cm/s
        else:
            main.set_alternative_parameters(climb_rate=-0.6)  # decent with 60cm/s

        # main.set_parameters(speed=0)  # should not be necessary, already called in start_mission()

    def continue_from_submission(self):
        self.start_mission()

    def continue_from_submission_error(self):
        self.start_mission()
