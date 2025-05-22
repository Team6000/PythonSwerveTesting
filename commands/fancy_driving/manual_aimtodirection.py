from __future__ import annotations

import math
import typing
import commands2
from wpimath.geometry import Rotation2d
from constants import AutoMovementConstants


class AimToDirectionConstants:
    kP = 0.002  # The P for the PID
    kUseSqrtControl = AutoMovementConstants.kUseSqrtControl # Gets it from constants file
    kMinTurnSpeed = 0.03  # Saying that if it wants to turn like 0.02 it's too slow as if it doesn't move.
    kAngleToleranceDegrees = 2.0  # How many degrees of when it thinks it is close enough
    kAngleVelocityToleranceDegreesPerSec = 50  # When the angular velocity is below 50 degrees per second it's not moving



class AimToDirection(commands2.Command):
    def __init__(self, degrees: float | typing.Callable[[], float], drivetrain, speed=1.0, fwd_speed=0.0):
        """
        :param degrees: What degree to aim to
        :param drivetrain: What drivetrain to drive
        :param speed: What speed to turn at
        :param fwd_speed: What forward speed to have

        Turns the robot to face the given location
        """

        super().__init__() # Initialize the Command database

        self.drivetrain = drivetrain # Initialize
        self.addRequirements(drivetrain) # Add the requirement for the command to run
        # Set speed to be the abs value of what the speed asked for is
        if abs(speed) < 1.0:
            self.speed = abs(speed)
        else:
            self.speed = 1.0
        # Initializes values
        self.targetDirection = None
        self.fwdSpeed = fwd_speed

        # Set the target degrees
        self.targetDegrees = degrees
        if degrees is None: # If it's none set it to the current degrees
            self.targetDegrees = lambda: self.drivetrain.getGyroHeading().degrees()
        elif not callable(degrees): # if it's not callable make it callable
            self.targetDegrees = lambda: degrees

    def initialize(self):
        """
        What runs when the function is started
        """
        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees()) # Sets the target direction

    def execute(self):
        # Figures out how many degrees are left to turn?
        currentDirection = self.drivetrain.getGyroHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()

        # do not drive left from 350 to 0 instead just turn right (do what is shorter).
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # Add PID for the turning (the closer you are to being done the slower it goes)
        turnSpeed = self.speed
        proportionalSpeed = AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed
        if turnSpeed > proportionalSpeed:
            turnSpeed = proportionalSpeed

        # If it wants to turn less then the min turn speed then it just moves the min turn speed.
        if turnSpeed < AimToDirectionConstants.kMinTurnSpeed and self.fwdSpeed == 0:
            turnSpeed = AimToDirectionConstants.kMinTurnSpeed

        # Now that it has the turn speed to turn with it can actually turn
        if degreesRemaining > 0: # If the degrees left is positive then give it
            self.drivetrain.ArcadeDrive(self.fwdSpeed, +turnSpeed)
        else:
            self.drivetrain.ArcadeDrive(self.fwdSpeed, -turnSpeed)  # otherwise, turn left

    def end(self, interrupted: bool):
        """
        When the command ends stop the drive train
        """
        self.drivetrain.ArcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """
        What tells the command when it is naturally done
        """
        if self.fwdSpeed != 0:
            return False   # if someone wants us to drive forward while aiming, then it's never naturally finished

        # calculates degrees remaining
        currentDirection = self.drivetrain.getGyroHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # if we are pretty close to the direction we wanted, the command finished
        if abs(degreesRemaining) < AimToDirectionConstants.kAngleToleranceDegrees:
            turnVelocity = self.drivetrain.getTurnRateDegreesPerSec()
            if abs(turnVelocity) < AimToDirectionConstants.kAngleVelocityToleranceDegreesPerSec:
                return True
        return False # it's not done