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
            derivative=1/2,
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
            tol=0.8
        )
        self.zeroZ = self.drivetrain.getGyroAngleZ()
        self.acComp = 0
        self.drift = self.drivetrain.getGyroAngleZ()
        
    def reset(self):
        self.onRamp = False


    def run(self):
        #define forward constant for now
        forward = 0.7

        # ramp pitch stop condition
        pitch = self.drivetrain.getGyroAngleY() - self.drift
        if pitch > 4.8:
            self.onRamp = True
        elif self.onRamp:
            self.drivetrain.move(0, 0)
            return


        # general corrective PID steering
        lTravel = self.drivetrain.getLEncoderDistance() + self.acComp
        rTravel = self.drivetrain.getREncoderDistance() + self.acComp
        # avgDistance = self.drivetrain.getAvgDistanceTravelled()
        diff = lTravel - rTravel
        pid_diff = self.dir_pid_controller.calculate(diff)
        rotate = max(-RC.maxTurnSpeed, min(RC.maxTurnSpeed, pid_diff))
        # forward = self.fwd_pid_controller.calculate(avgDistance)

        # print(f"Left traveled:{lTravel}, Right traveled:{rTravel}, Avg traveled:{self.drivetrain.getAvgDistanceTravelled()}")
        if self.dir_pid_controller.atSetpoint():
            rotate = 0
            print("correction not applied")
        # rotate = diff * self.kp
        self.drivetrain.move(rotate, forward)

        # accidental Z rotation control
        zdiff = self.accidental_pid_controller.calculate(
            self.drivetrain.getGyroAngleZ() - self.zeroZ
        )
        zdiff = max(-RC.maxTurnSpeed, min(zdiff, RC.maxTurnSpeed))
        if not self.accidental_pid_controller.atSetpoint():
            self.drivetrain.move(zdiff, forward)
            if zdiff > 0:
                self.acComp += abs(zdiff)
            else:
                self.acComp -= abs(zdiff)
        print(f"{forward=}, {rotate=}, distance: {self.drivetrain.getAvgDistanceTravelled()}, difference: {diff}")