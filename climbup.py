from drivetrain import Drivetrain as DT
from robotconstants import RobotConstants as RC
from wpimath.controller import PIDController
from autoroutine import AutoRoutine

class climbup(AutoRoutine):
    def __init__(self, drivetrain: DT):
        self.drivetrain = drivetrain
        self.tolerance = 3

    def run(self):
        pass
