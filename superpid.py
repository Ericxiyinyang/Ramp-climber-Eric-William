from wpimath.controller import PIDController

class AIOPID:
    def __init__(self, prop, integral, derivative, setPoint, tol):
        self.pid_controller = PIDController(
            prop,
            integral,
            derivative
        )
        self.pid_controller.setSetpoint(setPoint)
        self.pid_controller.setTolerance(tol)

    def setIntgRange(self, min, max):
        self.pid_controller.setIntegratorRange(min, max)

    def setSetpoint(self, setPoint):
        self.pid_controller.setSetpoint(setPoint)

    def setTolerance(self, tol):
        self.pid_controller.setTolerance(tol)

    def calculate(self, current_reading):
        return self.pid_controller.calculate(current_reading)

    def atSetpoint(self):
        return self.pid_controller.atSetpoint()