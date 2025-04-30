from __future__ import annotations

import commands2
import typing

from wpimath.geometry import Pose2d, Rotation2d
from commands2 import RunCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SmartDashboard

from constants import OIConstants
from subsystems.drivesubsystem import DriveSubsystem

from commands.reset_xy import ResetXY, ResetSwerveFront
from pathplannerlib.auto import AutoBuilder
from commands.fancy_driving.pathplanner_to_pose import PathtoPose


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()




        # The driver's controller
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.autoChooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Configure default command for driving using sticks
        from commands.swervedrive import SwerveDrive
        self.robotDrive.setDefaultCommand(
            SwerveDrive(
                self.robotDrive,
                forwardSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        #TODO: ADD DRIVE FORWARD BUTTON

        xButton = self.driverController.button(XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))

        yButton = self.driverController.button(XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

        rbButton = self.driverController.button(XboxController.Button.kRightBumper)
        rbButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))

        #TODO: TEST: WHICH ONE IS THE RIGHT WAY? DO ANY WORK??
        aButton = self.driverController.button(XboxController.Button.kA)
        #move_to_pose = PathtoPose(Pose2d(10,10,Rotation2d(0)), self.robotDrive)
        move_to_pose = RunCommand(lambda: PathtoPose(Pose2d(10,10,Rotation2d(0)), self.robotDrive))
        aButton.whileTrue(move_to_pose)



    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        # """
        return self.autoChooser.getSelected()



    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
