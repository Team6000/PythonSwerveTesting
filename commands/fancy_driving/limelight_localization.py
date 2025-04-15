from commands2 import Command
from subsystems.limelightsubsystem import LimelightSubsystem
from wpimath.geometry import Pose2d, Rotation2d

class SetLimelightPoseCommand(Command):
    def __init__(self, limelight: LimelightSubsystem, drivetrain):
        super().__init__()
        self.limelight = limelight
        self.drivetrain = drivetrain

        self.addRequirements(drivetrain)

    def initialize(self):
        pose_array = self.limelight.getBotPose()
        tag_count = self.limelight.getBotPoseTagCount()
        if pose_array is None:
            print("[SetLimelightPoseCommand] No valid pose received from Limelight")
            return
        if tag_count < 1: # MAYBE MAKE IT LESS THAN 2?
            print("[SetLimelightPoseCommand] Not enough tags")
            return
        if abs(self.drivetrain.getTurnRateDegreesPerSec()) > 360:
            print("[SetLimelightPoseCommand] Moving too fast")
            return

        pose = Pose2d(pose_array[0], pose_array[1], Rotation2d(pose_array[5]))
        self.drivetrain.resetOdometry(pose)
        print(f"[SetLimelightPoseCommand] Reset pose to: {pose}")

    def isFinished(self):
        return True
