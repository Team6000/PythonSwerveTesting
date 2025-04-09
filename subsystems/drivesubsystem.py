import math
import typing
import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import SmartDashboard, Field2d, DriverStation

import constants
from constants import DriveConstants, ModuleConstants
import swerveutils
from subsystems.swervemodule_cancoder import SwerveModule_CANCoder
from subsystems.swervemodule import SwerveModule
from rev import SparkMax
import navx
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

#TODO: Y WAS REVERSE IN POSE

class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()

        # Makes sure that if a maxSpeedScaleFactor is given it is a function
        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        # Create 4 Swerve Modules
        self.frontLeft = SwerveModule_CANCoder(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftRotationOffset,
            DriveConstants.kFrontLeftCANCoderID,
            turnMotorInverted=ModuleConstants.kfrontLeft_turn_inverted,
            driveMotorInverted= ModuleConstants.kfrontLeft_drive_inverted,
            encoderInverted=ModuleConstants.kfrontLeft_encoder_inverted,
            motorControllerType=SparkMax,
        )

        self.frontRight = SwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightRotationOffset,
            turnMotorInverted=ModuleConstants.kfrontRight_turn_inverted,
            driveMotorInverted=ModuleConstants.kfrontRight_drive_inverted,
            encoderInverted=ModuleConstants.kfrontRight_encoder_inverted,
            motorControllerType=SparkMax,
        )

        self.backLeft = SwerveModule(
            DriveConstants.kBackLeftDrivingCanId,
            DriveConstants.kBackLeftTurningCanId,
            DriveConstants.kBackLeftRotationOffset,
            turnMotorInverted=ModuleConstants.kbackLeft_turn_inverted,
            driveMotorInverted=ModuleConstants.kbackLeft_drive_inverted,
            encoderInverted=ModuleConstants.kbackLeft_encoder_inverted,
            motorControllerType=SparkMax,
        )

        self.backRight = SwerveModule(
            DriveConstants.kBackRightDrivingCanId,
            DriveConstants.kBackRightTurningCanId,
            DriveConstants.kBackRightRotationOffset,
            turnMotorInverted=ModuleConstants.kbackRight_turn_inverted,
            driveMotorInverted=ModuleConstants.kbackRight_drive_inverted,
            encoderInverted=ModuleConstants.kbackRight_encoder_inverted,
            motorControllerType=SparkMax,

        )

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroState = "ok"

        # Slew rate filter variables for controlling lateral acceleration
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(0, 0, 0))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # PathPlanner Setup:
        # config = RobotConfig.fromGUISettings()
        #
        # AutoBuilder.configure(
        #     self.getPose,  # Robot pose supplier
        #     self.resetOdometry,  # Method to reset odometry (will be called if your auto has a starting pose)
        #     self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        #     lambda speeds, feedforwards: self.driveRobotRelative(speeds),
        #     # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
        #     PPHolonomicDriveController(
        #         # PPHolonomicController is the built-in path following controller for holonomic drive trains
        #         PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
        #         PIDConstants(5.0, 0.0, 0.0)  # Rotation PID constants
        #     ),
        #     config,  # The robot configuration
        #     self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
        #     self  # Reference to this subsystem to set requirements
        # )



    def periodic(self) -> None:

        # Update the odometry of the robot
        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        # Puts info of pose on SmartDashboard
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())

        # Put the encoders  of all four modules
        SmartDashboard.putNumber("fl", ((self.frontLeft.turningEncoder.getPosition() * 180 / math.pi) - constants.DriveConstants.kFrontLeftRotationOffset))
        SmartDashboard.putNumber("fr", ((self.frontRight.turningEncoder.getPosition() * 180 / math.pi) - constants.DriveConstants.kFrontRightRotationOffset))
        SmartDashboard.putNumber("bl", ((self.backLeft.turningEncoder.getPosition() * 180 / math.pi) - constants.DriveConstants.kBackLeftRotationOffset))
        SmartDashboard.putNumber("br", ((self.backRight.turningEncoder.getPosition() * 180 / math.pi) - constants.DriveConstants.kBackRightRotationOffset))

        self.field.setRobotPose(pose)

    def getPoseHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0

        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()


    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            newPose,
        )
        self.odometryHeadingOffset += dRot

    def stop(self):
        self.drive(0, 0, 0, False, False)


    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        rateLimit: bool,
        square: bool = False
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """

        if square:
            rot = rot * abs(rot)
            norm = math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm

        if (xSpeed != 0 or ySpeed != 0) and self.maxSpeedScaleFactor is not None:
            norm = math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed)
            scale = abs(self.maxSpeedScaleFactor() / norm)
            if scale < 1:
                xSpeed = xSpeed * scale
                ySpeed = ySpeed * scale

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    DriveConstants.kDirectionSlewRate / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(
                inputTranslationDir, self.currentTranslationDir
            )
            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(
                    inputTranslationMag
                )

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(
                        self.currentTranslationDir + math.pi
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(
                self.currentTranslationDir
            )
            ySpeedCommanded = self.currentTranslationMag * math.sin(
                self.currentTranslationDir
            )
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = self.currentRotation * DriveConstants.kMaxAngularSpeed

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                self.getGyroHeading(),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.backLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.backRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
        self,
        desiredStates: typing.Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()

    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot, tries to be smart when gyro is disconnected

        :returns: the robot's heading as Rotation2d
        """
        now = wpilib.Timer.getFPGATimestamp()
        past = self._lastGyroAngleTime
        state = "ok"

        if not self.gyro.isConnected():
            state = "disconnected"
        else:
            if self.gyro.isCalibrating():
                state = "calibrating"
            self._lastGyroAngle = self.gyro.getAngle()
            self._lastGyroAngleTime = now

        if state != self._lastGyroState:
            SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
            self._lastGyroState = state

        return Rotation2d.fromDegrees(self._lastGyroAngle * DriveConstants.kGyroReversed)


    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * DriveConstants.kGyroReversed


    def getTurnRateDegreesPerSec(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.getTurnRate() * 180 / math.pi

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Returns the current robot-relative chassis speeds.

        This method uses the current wheel speeds and the robot's heading to calculate
        the robot-relative chassis speeds.

        :returns: The current robot-relative chassis speeds.
        """
        # Get the current states of the swerve modules
        fl_state = self.frontLeft.getState()
        fr_state = self.frontRight.getState()
        rl_state = self.backLeft.getState()
        rr_state = self.backRight.getState()

        # Convert the swerve module states into chassis speeds (robot-relative)
        # Use SwerveDrive4Kinematics to calculate the ChassisSpeeds
        chassis_speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
            fl_state, fr_state, rl_state, rr_state
        )

        return chassis_speeds

    def driveRobotRelative(self, chassis_speeds: ChassisSpeeds):
        """Outputs commands to the robot's drive motors given robot-relative chassis speeds.

        This function uses the robot-relative chassis speeds and converts them into individual
        swerve module states using the kDriveKinematics.

        :param chassis_speeds: The robot-relative chassis speeds (vx, vy, omega)
        """
        # Use kDriveKinematics to convert ChassisSpeeds to individual module states
        module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)

        # Apply module states to the swerve modules
        self.frontLeft.setDesiredState(module_states[0])
        self.frontRight.setDesiredState(module_states[1])
        self.backLeft.setDesiredState(module_states[2])
        self.backRight.setDesiredState(module_states[3])

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed