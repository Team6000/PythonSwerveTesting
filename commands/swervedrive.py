from __future__ import annotations
from wpimath import applyDeadband
import commands2
 
class SwerveDrive(commands2.Command):
    """
    Command to run the Swerve Drive robot and make the robot drive.
    """
    def __init__(self, drivetrain, forwardSpeed, leftSpeed, rotationSpeed, deadband=0, **kwargs):
        """
        Drive the robot at `driveSpeed` and `rotationSpeed` until this command is terminated.
        """
        super().__init__()

        self.forwardSpeed = forwardSpeed
        if not callable(forwardSpeed):
            self.forwardSpeed = lambda: forwardSpeed

        self.leftSpeed = leftSpeed
        if not callable(leftSpeed):
            self.leftSpeed = lambda: leftSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        assert deadband >= 0, f"deadband={deadband} is not positive"
        self.deadband = deadband

        self.drivetrain = drivetrain
        self.kwargs = kwargs

        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        self.drivetrain.drive(
            applyDeadband(self.forwardSpeed(), self.deadband),
            applyDeadband(self.leftSpeed(), self.deadband),
            applyDeadband(self.rotationSpeed(), self.deadband),
            **self.kwargs
        )

    def end(self, interrupted: bool):
        self.drivetrain.stop()  # stop immediately if command is ending
