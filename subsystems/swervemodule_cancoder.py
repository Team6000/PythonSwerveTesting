from wpimath.geometry import Rotation2d

from swervemodule import SwerveModule
from rev import SparkMax
from phoenix6.hardware import CANcoder

class SwerveModule_CANCoder(SwerveModule):
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            chassisAngularOffset: float, # TODO: Don't understand
            cancoder_id: int,
            turnMotorInverted=False,
            driveMotorInverted=False,
            encoderInverted=False,
            motorControllerType=SparkMax,
    ) :
        super().__init__(drivingCANId,turningCANId,chassisAngularOffset,turnMotorInverted,driveMotorInverted,encoderInverted,motorControllerType)

        # Gets CANCoder Abs Encoder Position
        self.cancoder_id = cancoder_id
        self.turning_AbsEncoder = CANcoder(cancoder_id)

        # Sets the position of the relative encoder to the abs encoder
        self.turningEncoder = self.turningSparkMax.getEncoder()
        self.turningEncoder.setPosition(self.turning_AbsEncoder.get_absolute_position())

        # Uses the new encoder position to write the initial desired state to the current angle
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())