from pathplannerlib.path import PathConstraints
import commands2
from pathplannerlib.auto import AutoBuilder
from wpimath.geometry import Pose2d
from subsystems.drivesubsystem import DriveSubsystem


# https://github.com/Team4169/Swerve2025/blob/master/robotcontainer.py

class PathToPoseConstants:
    maxVelocityMps = 3
    maxAccelerationMpsSq = 3
    maxAngularVelocityRps = 540
    maxAngularAccelerationRpsSq = 720

    constraints = PathConstraints(
        maxVelocityMps, maxAccelerationMpsSq,
        maxAngularVelocityRps, maxAngularAccelerationRpsSq
    )

# Option 1:
"""
class PathToPose(commands2.Command):
    def __init__(self, drive: DriveSubsystem, target_pose: Pose2d) -> None:
        super().__init__()

        self.drivetrain = drive
        self.command = None
        self.running = False
        self.addRequirements(self.drivetrain)
        self.target_pose = target_pose
        self.setName("PathToPoseOuter")

    def initialize(self) -> None:
        print("initialize...initialize...initialize...initialize...initialize...initialize...")
        self.running = True

        targetPose = self.target_pose

        self.command = AutoBuilder.pathfindToPose(
            targetPose,
            PathToPoseConstants.constraints
        )

        self.command.initialize()
        print("Debug 1")

    def execute(self) -> None:
        print("execute...execute...execute...execute...execute...execute...execute...")
        self.command.execute()

        if self.command.isFinished():
            self.command.end(False)
            self.running = False
            print("Debug 2")
        print("Debug 3")

    def isFinished(self) -> bool:
        if self.command:
            print("Debug 4")
            if self.command.isFinished():
                print("Debug 5")
                return True
        return False


    def end(self, interrupted: bool):
        print("Debug: 6")
        if self.command:
            print("Debug: 7")
            self.command.cancel()
            
            
"""

# Option 2:

class PathToPose(commands2.Command):
    def __init__(self, drive: DriveSubsystem, target_pose: Pose2d) -> None:
        super().__init__()

        self.drivetrain = drive
        self.command = None
        self.target_pose = target_pose
        self.setName("PathToPoseOuter")
        self.addRequirements(self.drivetrain) # TODO: TRY REMOVING IF DOESN'T WORK
        print("Debug 0")
        

    def initialize(self) -> None:
        print("Initializing PathToPose")

        self.command = AutoBuilder.pathfindToPose(
            self.target_pose,
            PathToPoseConstants.constraints
        )
        self.command.schedule()# Schedule the pathfinding command
        print("Debug 1")

    def isFinished(self) -> bool:
        print("Debug 2")
        return self.command is not None and self.command.isFinished()

    def end(self, interrupted: bool) -> None:
        print("Debug 3")
        if self.command:
            self.command.cancel()
