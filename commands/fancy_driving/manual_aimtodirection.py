from __future__ import annotations

import math
import typing
import commands2
from wpimath.geometry import Rotation2d
from constants import AutoMovementConstants

# TODO: Just change this to PathPlanner with same x,y and different rot

class AimToDirectionConstants:
    kP = 0.002  # 0.002 is the default, but you must calibrate this to your robot
    kUseSqrtControl = AutoMovementConstants.kUseSqrtControl
    kMinTurnSpeed = 0.03  # turning slower than this is unproductive for the motor (might not even spin)
    kAngleToleranceDegrees = 2.0  # plus minus 2 degrees is "close enough"
    kAngleVelocityToleranceDegreesPerSec = 50  # velocity under 100 degrees/second is considered "stopped"



class AimToDirection(commands2.Command):
    def __init__(self, degrees: float | typing.Callable[[], float], drivetrain, speed=1.0, fwd_speed=0.0):
        super().__init__()

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        if abs(speed) < 1.0:
            self.speed = abs(speed)
        else:
            self.speed = 1.0
        self.targetDirection = None
        self.fwdSpeed = fwd_speed

        # setting the target angle in a way that works for all cases
        self.targetDegrees = degrees
        if degrees is None:
            self.targetDegrees = lambda: self.drivetrain.getGyroHeading().degrees()
        elif not callable(degrees):
            self.targetDegrees = lambda: degrees

    def initialize(self):
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())

    def execute(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getGyroHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed and self.fwdSpeed == 0:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed  # but not too small

        # 3. act on it! if target angle is on the right, turn right
        if degreesRemaining > 0:
            self.drivetrain.ArcadeDrive(self.fwdSpeed, +turnSpeed)
        else:
            self.drivetrain.ArcadeDrive(self.fwdSpeed, -turnSpeed)  # otherwise, turn left

    def end(self, interrupted: bool):
        self.drivetrain.ArcadeDrive(0, 0)

    def isFinished(self) -> bool:
        if self.fwdSpeed != 0:
            return False   # if someone wants us to drive forward while aiming, then we are never finished

        currentDirection = self.drivetrain.getGyroHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # if we are pretty close to the direction we wanted, consider the command finished
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            turnVelocity = self.drivetrain.getTurnRateDegreesPerSec()
            if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec: #TODO: WHY SO HIGH
                return True
        return False