import math

from wpimath.geometry import Rotation2d

from constants import getSwerveTurningMotorConfig
from subsystems.swervemodule import SwerveModule
from rev import SparkMax, SparkBase
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
            motorControllerType=SparkMax, # Not used but here for easy changes later
    ) :
        """
        Creates a SwerveModule the same as the super except that this is for a CANCoder so it sets the
        relative encoder to that as there is no abs encoder plugged in.
        """
        super().__init__(drivingCANId, turningCANId, moduleRotationOffset, turnMotorInverted, driveMotorInverted, encoderInverted, motorControllerType)

        # reconfigures the turning motor controller
        self.turningSparkMax.configure(
            getSwerveTurningMotorConfig(turnMotorInverted, encoderInverted, abs_enc=False),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        # Creates the turning PID controller
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()


        # Gets CANCoder Abs Encoder
        self.canCoder_id = canCoder_id
        self.turning_AbsEncoder = CANcoder(canCoder_id)

        # Sets the position of the relative encoder to the abs encoder.
        self.turningEncoder = self.turningSparkMax.getEncoder()
        rotation_value = self.turning_AbsEncoder.get_absolute_position().value # gets value
        deg_value = rotation_value*360 # converts to degrees
        adj_deg_value = deg_value - moduleRotationOffset # applies offset
        rad_value = -adj_deg_value * math.pi/180 # converts to radians
        self.turningEncoder.setPosition(rad_value) # sets the position

        # Uses the new encoder position to write the initial desired state to the current angle
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition() - self.moduleRotationOffset)