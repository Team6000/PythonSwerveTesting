from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
import commands2



class PathToPathConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720


class PathtoPose(commands2.Command):
    def __init__(self, target_path, drivetrain):
        super().__init__()
        self.target_path = target_path
        self.drivetrain = drivetrain
        self.command = None

    def initialize(self):
        path = PathPlannerPath.fromPathFile(self.target_path)
        constraints = PathConstraints(
            PathToPathConstants.maxVelocityMps, PathToPathConstants.maxAccelerationMpsSq,
            PathToPathConstants.maxAngularVelocityRps, PathToPathConstants.maxAngularAccelerationRpsSq
        )
        self.command = AutoBuilder.pathfindThenFollowPath(path, constraints)
        self.command.initialize()

    def execute(self):
        if self.command:
            self.command.execute()

    def isFinished(self):
        if self.command:
            self.command.isFinished()
        return True # Just incase no command was created


    def end(self, interrupted: bool):
        if self.command:
            self.command.end(interrupted)
        self.drivetrain.stop()
