from __future__ import annotations
import commands2

from wpimath.geometry import Rotation2d, Pose2d


class ResetXY(commands2.Command):
    def __init__(self, x, y, headingDegrees, drivetrain):
        """
        Reset the starting (X, Y) and heading (in degrees) of the robot to where they should be.
        :param x: X
        :param y: Y
        :param headingDegrees: heading
        :param drivetrain: drivetrain to set it
        """
        super().__init__()

        # Initialize Variables
        self.drivetrain = drivetrain
        self.position = Pose2d(x,y, Rotation2d.fromDegrees(headingDegrees))
        self.addRequirements(drivetrain)

    def initialize(self):
        self.drivetrain.resetOdometry(self.position) # Resets the drive train

    def isFinished(self) -> bool:
        return True  # It is a instant command

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """


class ResetSwerveFront(commands2.Command):
    def __init__(self, drivetrain):
        """
        :param drivetrain: The Drivetrain to reset
        """
        super().__init__()

        # Initializes variable
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        curr_pose = self.drivetrain.getPose() # Current position
        pose = Pose2d(curr_pose.x, curr_pose.y, Rotation2d(0)) # Gets the pose to set it
        self.drivetrain.resetOdometry(pose) # sets the pose

    def isFinished(self) -> bool:
        return True  # Instant command ends immediately

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """
