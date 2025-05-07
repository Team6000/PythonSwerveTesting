from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
import commands2
from wpimath.geometry import Pose2d, Rotation2d
from subsystems.drivesubsystem import DriveSubsystem

# DON'T USE. USE THE MANUAL ONE
class PathAimToDirectionConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720

    constraints = PathConstraints(
        maxVelocityMps, maxAccelerationMpsSq,
        maxAngularVelocityRps, maxAngularAccelerationRpsSq
    )


class PathAimToDirection(commands2.Command):
    def __init__(self, degrees: float, drive: DriveSubsystem):
        super().__init__()

        self.drivetrain = drive
        self.command = None
        self.commanded_pose = None
        self.target_degrees = degrees
        self.setName("PathPlannerAimToDirection")
        self.addRequirements(self.drivetrain)


    def initialize(self):
        current_pose = self.drivetrain.getPose()
        self.commanded_pose = Pose2d(current_pose.x,current_pose.y,Rotation2d.fromDegrees(self.target_degrees))

        self.command = AutoBuilder.pathfindToPose(
            self.commanded_pose,
            PathAimToDirectionConstants.constraints,
            goal_end_vel=0,
        )
        self.command.schedule()

    def isFinished(self):
        if self.command:
            return self.command.isFinished()
        return False

    def end(self, interrupted: bool):
        if self.command:
            self.command.cancel()
