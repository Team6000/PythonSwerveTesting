from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
import commands2
from wpimath.geometry import Pose2d, Rotation2d


class PathAimToDirectionConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720


class PathAimToDirection(commands2.Command):
    def __init__(self, degrees: float, drivetrain):
        super().__init__()
        self.target_degrees = degrees
        self.drivetrain = drivetrain
        self.command = None
        self.commanded_pose = None

    def initialize(self):
        constraints = PathConstraints(
            PathAimToDirectionConstants.maxVelocityMps, PathAimToDirectionConstants.maxAccelerationMpsSq,
            PathAimToDirectionConstants.maxAngularVelocityRps, PathAimToDirectionConstants.maxAngularAccelerationRpsSq
        )

        current_pose = self.drivetrain.getPose()
        self.commanded_pose = Pose2d(current_pose.x,current_pose.y,Rotation2d.fromDegrees(self.target_degrees))

        self.command = AutoBuilder.pathfindToPose(
            self.commanded_pose,
            constraints,
            goal_end_vel=0,
        )
        self.command.initialize()

    def execute(self):
        if self.command:
            self.command.execute()

    def isFinished(self):
        if self.command:
            return self.command.isFinished()
        return True  # Just in case command is None

    def end(self, interrupted: bool):
        if self.command:
            self.command.end(interrupted)
        self.drivetrain.stop()
