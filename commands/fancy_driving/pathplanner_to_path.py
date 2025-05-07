from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
import commands2
from subsystems.drivesubsystem import DriveSubsystem


class PathToPathConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720

    constraints = PathConstraints(
        maxVelocityMps, maxAccelerationMpsSq,
        maxAngularVelocityRps, maxAngularAccelerationRpsSq
    )


class PathtoPath(commands2.Command):
    def __init__(self, target_path, drive: DriveSubsystem) -> None:
        super().__init__()

        self.drivetrain = drive
        self.command = None
        self.target_path = target_path
        self.setName("PathToPathOuter")
        self.addRequirements(self.drivetrain)


    def initialize(self) -> None:
        path = PathPlannerPath.fromPathFile(self.target_path)
        self.command = AutoBuilder.pathfindThenFollowPath(
            path,
            PathToPathConstants.constraints
        )
        self.command.schedule()

    def isFinished(self) -> bool:
        if self.command:
            self.command.isFinished()
        return True

    def end(self, interrupted: bool) -> None:
        if self.command:
            self.command.cancel()
