import math

from wpimath.geometry import Rotation2d

from swervemodule import SwerveModule
from rev import SparkMax
from phoenix6.hardware import CANcoder

class SwerveModule_CANCoder(SwerveModule):
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            moduleRotationOffset: float,
            canCoder_id: int,
            turnMotorInverted=False,
            driveMotorInverted=False,
            encoderInverted=False,
            motorControllerType=SparkMax,
    ) :
        super().__init__(drivingCANId, turningCANId, moduleRotationOffset, turnMotorInverted, driveMotorInverted, encoderInverted, motorControllerType)

        # Gets CANCoder Abs Encoder Position
        self.canCoder_id = canCoder_id
        self.turning_AbsEncoder = CANcoder(canCoder_id)

        # Sets the position of the relative encoder to the abs encoder
        self.turningEncoder = self.turningSparkMax.getEncoder()

        rotation_value = self.turning_AbsEncoder.get_absolute_position().value
        rad_value = rotation_value * math.tau
        self.turningEncoder.setPosition(rad_value)

        # Uses the new encoder position to write the initial desired state to the current angle
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition() - self.moduleRotationOffset)