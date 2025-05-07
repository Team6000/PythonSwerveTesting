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
        self.thecommand = None

    def initialize(self):
        print("Debug 3")
        constraints = PathConstraints(
            PathToPoseConstants.maxVelocityMps, PathToPoseConstants.maxAccelerationMpsSq,
            PathToPoseConstants.maxAngularVelocityRps, PathToPoseConstants.maxAngularAccelerationRpsSq
        )

        self.thecommand = AutoBuilder.pathfindToPose(
            self.target_pose,
            constraints,
            goal_end_vel=self.goal_end_vel,
        )
        print("Debug 4")

    def return_command(self):
        print("Debug 1")
        self.initialize()
        return self.thecommand

    def execute(self):
        print("Debug 2")
        pass

    def isFinished(self):
        if self.thecommand:
            return self.thecommand.isFinished()
        return True  # Just in case command is None

    def end(self, interrupted: bool):
        if self.thecommand:
            self.thecommand.end(interrupted)
        self.drivetrain.stop()