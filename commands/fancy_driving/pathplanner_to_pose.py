from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
import commands2

class PathToPoseConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720


class PathtoPose(commands2.Command):
    def __init__(self, target_pose, drivetrain, goal_end_vel=0):
        super().__init__()
        self.target_pose = target_pose
        self.drivetrain = drivetrain
        self.goal_end_vel = goal_end_vel
        self.command = None

    def initialize(self):
        constraints = PathConstraints(
            PathToPoseConstants.maxVelocityMps, PathToPoseConstants.maxAccelerationMpsSq,
            PathToPoseConstants.maxAngularVelocityRps, PathToPoseConstants.maxAngularAccelerationRpsSq
        )

        self.command = AutoBuilder.pathfindToPose(
            self.target_pose,
            constraints,
            goal_end_vel=self.goal_end_vel,
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