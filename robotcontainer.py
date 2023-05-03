import wpilib as wp
from climbup import climbup
from drivetrain import Drivetrain as DT

class RobotContainer:
    def __init__(self):
        self.controller = wp.Joystick(0)
        self.drivetrain = DT()
        self.chooser = wp.SendableChooser()
        self._configure()

    def _configure(self):
        self.chooser.setDefaultOption("Climb Up", climbup(self.drivetrain))
        wp.SmartDashboard.putData(self.chooser)

    def get_autonomous(self):
        return self.chooser.getSelected()