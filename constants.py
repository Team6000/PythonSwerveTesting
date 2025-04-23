# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""



import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from rev import SparkBase, SparkBaseConfig, ClosedLoopConfig


class NeoMotorConstants:
    kFreeSpeedRpm = 5676


class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = math.tau  # radians per second

    kDirectionSlewRate = 1.2  # radians per second
    kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    kTrackWidth = units.inchesToMeters(22.5)
    # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(22.5)

    # Distance between front and back wheels on robot
    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)



    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 11
    kFrontLeftTurningCanId = 12
    kFrontLeftCANCoderID = 13
    kFrontLeftRotationOffset = 200.92

    kFrontRightDrivingCanId = 21
    kFrontRightTurningCanId = 22
    kFrontRightCANCoderID = None
    kFrontRightRotationOffset = 0

    kBackLeftDrivingCanId = 31
    kBackLeftTurningCanId = 32
    kBackLeftCANCoderID = None
    kBackLeftRotationOffset = 0

    kBackRightDrivingCanId = 41
    kBackRightTurningCanId = 42
    kBackRightCANCoderID = None
    kBackRightRotationOffset = 0




    kGyroReversed = -1  # can be +1 if not flipped (affects field-relative driving)


def getSwerveDrivingMotorConfig(drivingMotorInverted: bool) -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.inverted(drivingMotorInverted)
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig


def getSwerveTurningMotorConfig(turnMotorInverted: bool, encoderInverted: bool, abs_enc = True) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(encoderInverted)
    turningConfig.encoder.positionConversionFactor(ModuleConstants.kTurningAbsEncoderPositionFactor)
    turningConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningAbsEncoderVelocityFactor)
    if abs_enc:
        turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    else:
        turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    return turningConfig


class ModuleConstants:
    #TODO: DRIVE INVERTED: DO WHEN WE FIX SWERVE
    kfrontLeft_drive_inverted = False
    kfrontLeft_turn_inverted = True
    kfrontRight_drive_inverted = False
    kfrontRight_turn_inverted = True
    kbackLeft_drive_inverted = False
    kbackLeft_turn_inverted = True
    kbackRight_drive_inverted = False
    kbackRight_turn_inverted = True


    kfrontLeft_encoder_inverted = False
    kfrontRight_encoder_inverted = True
    kbackLeft_encoder_inverted = True
    kbackRight_encoder_inverted = True


    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.116
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDrivingMotorReduction = (50.0/16.0) * (17.0 /27.0) * (45.0 / 15.0)

    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0  # meters per second

    kTurningEncoderPositionFactor = math.tau  # radian
    kTurningEncoderVelocityFactor = math.tau / 60.0  # radians per second

    kTurningMotorGearReduction = (14.0 / 50) * (10.0 / 60.0)
    kTurningAbsEncoderPositionFactor = math.tau * kTurningMotorGearReduction # Converts to Radians
    kTurningAbsEncoderVelocityFactor = (math.tau * kTurningMotorGearReduction) / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radian

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1  # can be dialed down if you see oscillations in the turning motor
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    kTurningMotorIdleMode = SparkBase.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp # TODO NERIYA HAD 40

    kDrivingMinSpeedMetersPerSecond = 0.01


class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05