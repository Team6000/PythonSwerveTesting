from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
import commands2

import constants
from subsystems.drivesubsystem import DriveSubsystem


class PathToPathConstants:
    """
    Constants for PathtoPath Command
    """
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720

    constraints = PathConstraints(
        maxVelocityMps, maxAccelerationMpsSq,
        maxAngularVelocityRps, maxAngularAccelerationRpsSq
    )


class PathToPath(commands2.Command):
    """
    Command to make a Path to an Auto path
    """
    def __init__(self, target_path: str, drive: DriveSubsystem) -> None:
        super().__init__()

        # Initialize variables
        self.drivetrain = drive
        self.command = None
        self.target_path = target_path
        self.setName("PathToPathOuter")
        self.addRequirements(self.drivetrain)

    def initialize(self) -> None:
        if constants.in_field(self.drivetrain.getPose()): # If the robot is in the field (must be for it to run)
            path = PathPlannerPath.fromPathFile(self.target_path) # Makes the command
            self.command = AutoBuilder.pathfindThenFollowPath(
                path,
                PathToPathConstants.constraints
            )
            self.command.schedule() # Schedules the command to run
        else:
            self.cancel() # If it's not in the field cancel this command

    def isFinished(self) -> bool:
        if self.command: # If the scheduled command is finished then this one is
            self.command.isFinished()
        return True

    def end(self, interrupted: bool) -> None:
        if self.command:
            self.command.cancel() # Cancel this command
