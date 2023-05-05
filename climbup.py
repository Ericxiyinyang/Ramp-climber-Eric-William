from drivetrain import Drivetrain as DT
from robotconstants import RobotConstants as RC
from wpimath.controller import PIDController
from autoroutine import AutoRoutine

class climbup(AutoRoutine):
    def __init__(self, drivetrain: DT):
        self.drivetrain = drivetrain
        self.tolerance = 3
        self.dir_pid_controller = PIDController(
            20,
            1/10,
            0
        )
        self.dir_pid_controller.setSetpoint(0)
        self.dir_pid_controller.setTolerance(.01)
        self.dir_pid_controller.setIntegratorRange(-.2, .2)
        self.onRamp = False

    def run(self):
        pitch = self.drivetrain.getGyroAngleY()
        if pitch > 5:
            self.onRamp = True
        elif self.onRamp:
            self.drivetrain.move(0, 0)
            return
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
