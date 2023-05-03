import wpilib as wp
from wpilib import Spark, Encoder
import wpilib.drive as drive
import romi
from robotconstants import RobotConstants as RC

class Drivetrain:
    def __init__(self):
        self.lMotor = Spark(0)
        self.rMotor = Spark(1)
        self.lEncoder = Encoder(4, 5)
        self.rEncoder = Encoder(6, 7)
        self.lEncoder.setDistancePerPulse(RC.distancePerTick)
        self.rEncoder.setDistancePerPulse(RC.distancePerTick)
        self.drivetrain = drive.DifferentialDrive(self.lMotor, self.rMotor)
        self.gyro = romi.RomiGyro()
        self.accelerometer = wp.BuiltInAccelerometer()

    def move(self, rotate, forward):
        self.drivetrain.arcadeDrive(rotate, forward)

    def getLEncoderDistance(self):
        return self.lEncoder.getDistance()

    def getREncoderDistance(self):
        return self.rEncoder.getDistance()

    def zeroEncoders(self):
        self.lEncoder.reset()
        self.rEncoder.reset()

    def getAvgDistanceTravelled(self):
        totalTravelled = self.lEncoder.getDistance() + self.rEncoder.getDistance()
        #convert to decimal precision
        return totalTravelled/2.0

    def getGyroAngleZ(self):
        """
        Give the twist of the robot
        :return: the current twist angle in degrees
        """
        return self.gyro.getAngleZ()

    def resetGyro(self):
        self.gyro.reset()
