from pathplannerlib.path import PathConstraints
import commands2
from pathplannerlib.auto import AutoBuilder
from wpimath.geometry import Pose2d
from subsystems.drivesubsystem import DriveSubsystem


class PathToPoseConstants:
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
        super().__init__()

        self.drivetrain = drive
        self.command = None
        self.target_pose = target_pose
        self.setName("PathToPoseOuter")
        self.addRequirements(self.drivetrain)


    def initialize(self) -> None:
        self.command = AutoBuilder.pathfindToPose(
            self.target_pose,
            PathToPoseConstants.constraints
        )
        self.command.schedule()# Schedule the pathfinding command

    def isFinished(self) -> bool:
        return self.command is not None and self.command.isFinished()

    def end(self, interrupted: bool) -> None:
        if self.command:
            self.command.cancel()