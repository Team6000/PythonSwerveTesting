from rev import SparkMax, SparkLowLevel, SparkBase
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
        motorControllerType = SparkMax # Not used but here for easy changes later
    ) -> None:
        """
        Creates a SwerveModule and configures the driving and turning motor,
        encoder, and PID controller.
        """


        self.moduleRotationOffset = moduleRotationOffset * math.pi / 180 # Wheel offsets degrees->rad
        self.desiredState = SwerveModuleState(0.0, Rotation2d()) # Initializes the desired state

        # Creates the 2 Sparks (Motor Controllers)
        self.drivingSparkMax = motorControllerType(
            drivingCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.turningSparkMax = motorControllerType(
            turningCANId, SparkLowLevel.MotorType.kBrushless
        )

        # Configures the SparkMax according to their config in the constants file
        self.drivingSparkMax.configure(
            getSwerveDrivingMotorConfig(driveMotorInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        self.turningSparkMax.configure(
            getSwerveTurningMotorConfig(turnMotorInverted, encoderInverted),
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters)

        # Setup encoders
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.turningEncoder = self.turningSparkMax.getAbsoluteEncoder()

        # Creates the PID controllers we will use to tell the robot to move
        self.drivingPIDController = self.drivingSparkMax.getClosedLoopController()
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()

        # Set the desired state to be at the current state of the robot (0, current_rotation)
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.drivingEncoder.setPosition(0) # Resets the driving encoder

    def getState(self) -> SwerveModuleState:
        """
        Returns the current state of the module.
        """
        # States are made up of speed of wheel rotation and angle it is at so we get those and return it
        return SwerveModuleState(
            self.drivingEncoder.getVelocity(),
            Rotation2d(self.turningEncoder.getPosition()),
        )

    def getPosition(self) -> SwerveModulePosition:
        """
        Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Similar to pose except that it uses the position of the driving encoder instead of speed.
        return SwerveModulePosition(
            self.drivingEncoder.getPosition(),
            Rotation2d(self.turningEncoder.getPosition()),
        )

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        Sets the desired state for the module. This is what actually runs the robot
        """
        # If it's asking you to move a very little amount just don't move
        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            # Checks if in break mode and if it is just stop the robot
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Have it creating a new one and setting it from the desireState so that if something needs to
        # be edited or reversed, etc. it can be easy. But these could 100% be combined.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle

        # Optimize the plan to get to the angle to avoid spinning further than 90 degrees.
        optimizedDesiredState = correctedDesiredState
        SwerveModuleState.optimize(
            optimizedDesiredState, Rotation2d(self.turningEncoder.getPosition())
        )

        # Let the motors to actually move
        self.drivingPIDController.setReference(
            optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity
        )
        self.turningPIDController.setReference(
            optimizedDesiredState.angle.radians(), SparkLowLevel.ControlType.kPosition
        )

        # Sets the classes desriedState to it.
        self.desiredState = desiredState

    def stop(self):
        """
        Stops the module
        """
        # Turns off both motors
        self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        self.turningPIDController.setReference(self.turningEncoder.getPosition(), SparkLowLevel.ControlType.kPosition)
        # If the desiredState's speed isn't zero set it to zero
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """
        Zeroes the SwerveModule encoders.
        """
        # Resets the encoder
        self.drivingEncoder.setPosition(0)