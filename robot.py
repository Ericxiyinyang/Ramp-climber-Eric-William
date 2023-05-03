import wpilib
import os
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from autoroutine import AutoRoutine
class MyRobot(TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()

        pass
    def robotPeriodic(self):
        '''This is called every cycle of the code. In general the code is loop
        through every .02 seconds.'''
        pass
    def autonomousInit(self):
        self.container.drivetrain.resetGyro()
        self.container.drivetrain.reset()
        '''This is called once when the robot enters autonomous mode.'''
        self.auto = self.container.get_autonomous()
        pass
    def autonomousPeriodic(self) -> AutoRoutine:
        self.auto.run() # this .run function is shared by both drivestraight and gyroTurn, so it is possible to do both
        '''This is called every cycle while the robot is in autonomous.'''
        pass
    def autonomousExit(self) -> None:
        #self.container.drivetrain.resetGyro()
        #self.container.drivetrain.reset()
        pass#gas
    def teleopInit(self):
        '''This is called once at the start of Teleop.'''
        pass

    def teleopPeriodic(self):
        forward = self.container.controller.getRawAxis(0)
        rotate = self.container.controller.getRawAxis(1)
        self.container.drivetrain.drive(rotate, forward)
        print(f"rotate:{rotate} forward:{forward}")
        '''This is called once every cycle during Teleop'''
        pass

if __name__ == "__main__":
    # If your ROMI isn't at the default address, set that here
    # If your ROMI isn't at the default address, set that here

    os.environ["HALSIMWS_HOST"] = "10.0.0.2"
    os.environ["HALSIMWS_PORT"] = "3300"
    wpilib.run(MyRobot)