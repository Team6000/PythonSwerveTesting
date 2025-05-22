from pathplannerlib.path import PathConstraints
import commands2
from pathplannerlib.auto import AutoBuilder
from wpimath.geometry import Pose2d

import constants
from subsystems.drivesubsystem import DriveSubsystem


class PathToPoseConstants:
    """
    Constants for PathToPose Command
    """
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720

    constraints = PathConstraints(
        maxVelocityMps, maxAccelerationMpsSq,
        maxAngularVelocityRps, maxAngularAccelerationRpsSq
    )


class PathToPose(commands2.Command):
    def __init__(self, drive: DriveSubsystem, target_pose: Pose2d) -> None:
        """
        :param drive: The drivetrain to drive
        :param target_pose: The taget Pose to drive to
        """
        super().__init__()

        # Initialize variables.
        self.drivetrain = drive
        self.command = None
        self.target_pose = target_pose
        self.setName("PathToPoseOuter")
        self.addRequirements(self.drivetrain)


    def initialize(self) -> None:
        if constants.in_field(self.drivetrain.getPose()): # Makes sure the robot in field.
            self.command = AutoBuilder.pathfindToPose( # Makes the command
                self.target_pose,
                PathToPoseConstants.constraints
            )
            self.command.schedule() # Schedule the pathfinding command
        else:
            print("you are outside. Get indoors fast") # If not in the field
            self.cancel()

    def isFinished(self) -> bool:
        if self.command:
            return self.command.isFinished() # If the command is finished
        return False

    def end(self, interrupted: bool) -> None:
        if self.command: # End the command
            self.command.cancel()