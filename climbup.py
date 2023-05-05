from drivetrain import Drivetrain as DT
from robotconstants import RobotConstants as RC
from wpimath.controller import PIDController
from autoroutine import AutoRoutine
from superpid import AIOPID

class climbup(AutoRoutine):
    def __init__(self, drivetrain: DT):
        self.drivetrain = drivetrain
        self.tolerance = 3
        self.dir_pid_controller = AIOPID(
            prop=20,
            integral=1/10,
            derivative=0,
            setPoint=0,
            tol=0.01
        )

        self.dir_pid_controller.setIntgRange(-.2, .2)
        self.onRamp = False

        self.accidental_pid_controller = AIOPID(
            prop=20,
            integral=1/10,
            derivative=0,
            setPoint=0,
            tol=0.01
        )
        self.zeroZ = self.drivetrain.getGyroAngleZ()

    def run(self):
        # ramp pitch stop condition
        pitch = self.drivetrain.getGyroAngleY()
        if pitch > 5:
            self.onRamp = True
        elif self.onRamp:
            self.drivetrain.move(0, 0)
            return

        # accidental Z rotation control


        # general corrective PID steering
        lTravel = self.drivetrain.getLEncoderDistance()
        rTravel = self.drivetrain.getREncoderDistance()
        # avgDistance = self.drivetrain.getAvgDistanceTravelled()
        diff = lTravel - rTravel
        pid_diff = self.dir_pid_controller.calculate(diff)
        rotate = max(-RC.maxTurnSpeed, min(RC.maxTurnSpeed, pid_diff))
        forward = 0.7
        # forward = self.fwd_pid_controller.calculate(avgDistance)

        # print(f"Left traveled:{lTravel}, Right traveled:{rTravel}, Avg traveled:{self.drivetrain.getAvgDistanceTravelled()}")
        if self.dir_pid_controller.atSetpoint():
            rotate = 0
            print("correction not applied")
        # rotate = diff * self.kp
        self.drivetrain.move(rotate, forward)
        print(f"{forward=}, {rotate=}, distance: {self.drivetrain.getAvgDistanceTravelled()}, difference: {diff}")
