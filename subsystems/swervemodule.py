from rev import SparkMax, SparkFlex, SparkLowLevel, SparkAbsoluteEncoder, SparkBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import math

from constants import ModuleConstants, getSwerveDrivingMotorConfig, getSwerveTurningMotorConfig


class SwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        moduleRotationOffset: float,
        turnMotorInverted = False,
        driveMotorInverted = False,
        encoderInverted = False,
        motorControllerType = SparkMax
    ) -> None:
        """Constructs a SwerveModule and configures the driving and turning motor,
        encoder, and PID controller.
        Encoder.
        """

        self.moduleRotationOffset = moduleRotationOffset * math.pi / 180 # degree -> radians
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        self.drivingSparkMax = motorControllerType(
            drivingCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.turningSparkMax = motorControllerType(
            turningCANId, SparkLowLevel.MotorType.kBrushless
        )

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        self.drivingSparkMax.configure(
            getSwerveDrivingMotorConfig(driveMotorInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.turningSparkMax.configure(
            getSwerveTurningMotorConfig(turnMotorInverted, encoderInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.

        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.turningEncoder = self.turningSparkMax.getAbsoluteEncoder() # Would this get the Redux

        self.drivingPIDController = self.drivingSparkMax.getClosedLoopController()
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()

        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.drivingEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(
            self.drivingEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getPosition() - self.moduleRotationOffset),
        )

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(
            self.drivingEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition() - self.moduleRotationOffset),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # if WPILib doesn't want us to move at all, don't bother to bring the wheels back to zero angle yet
            # (causes brownout protection when battery is lower: https://youtu.be/0Xi9yb1IMyA)
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(
            self.moduleRotationOffset
        )

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = correctedDesiredState
        SwerveModuleState.optimize(
            optimizedDesiredState, Rotation2d(self.turningEncoder.getPosition())
        )

        # Command driving and turning SPARKS MAX towards their respective set-points.
        self.drivingPIDController.setReference(
            optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity
        )
        self.turningPIDController.setReference(
            optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition
        )

        self.desiredState = desiredState

    def stop(self):
        """
        Stops the module in place to conserve energy and avoid unnecessary brownouts
        """
        self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        self.turningPIDController.setReference(self.turningEncoder.getPosition(), SparkLowLevel.ControlType.kPosition)
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self.drivingEncoder.setPosition(0)

