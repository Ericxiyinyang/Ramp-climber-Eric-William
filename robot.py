import wpilib
import os
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from autoroutine import AutoRoutine
class MyRobot(TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

    def robotPeriodic(self):
        '''This is called every cycle of the code. In general the code is loop
        through every .02 seconds.'''

    def autonomousInit(self):
        self.container.drivetrain.resetGyro()
        self.container.drivetrain.zeroEncoders()
        '''This is called once when the robot enters autonomous mode.'''
        self.auto = self.container.get_autonomous()

    def autonomousPeriodic(self) -> AutoRoutine:
        self.auto.run() # this .run function is shared by both drivestraight and gyroTurn, so it is possible to do both
        '''This is called every cycle while the robot is in autonomous.'''

    def autonomousExit(self) -> None:
        pass

    def teleopInit(self):
        pass
        '''This is called once when the robot enters teleop mode.'''


    def teleopPeriodic(self):
        forward = self.container.controller.getRawAxis(0)
        rotate = self.container.controller.getRawAxis(1)
        self.container.drivetrain.move(rotate, forward)
        print(f"rotate:{rotate} forward:{forward}")
        '''This is called once every cycle during Teleop'''


if __name__ == "__main__":
    # If your ROMI isn't at the default address, set that here
    # If your ROMI isn't at the default address, set that here

    os.environ["HALSIMWS_HOST"] = "10.0.0.2"
    os.environ["HALSIMWS_PORT"] = "3300"
    wpilib.run(MyRobot)