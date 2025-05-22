from __future__ import annotations
from wpimath import applyDeadband
import commands2
 
class SwerveDrive(commands2.Command):
    """
    Command to run the Swerve Drive robot and make the robot drive.
    """
    def __init__(self, drivetrain, forwardSpeed, leftSpeed, rotationSpeed, deadband=0, **kwargs):
        """
        Drive the robot at driveSpeed and rotationSpeed until this command is terminated.
        """
        super().__init__()

        # Initializes variables and makes them callable
        self.setName("Default Drive With Joystick")
        self.forwardSpeed = forwardSpeed
        if not callable(forwardSpeed):
            self.forwardSpeed = lambda: forwardSpeed

        self.leftSpeed = leftSpeed
        if not callable(leftSpeed):
            self.leftSpeed = lambda: leftSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        assert deadband >= 0, f"deadband={deadband} is not positive" # Makes sure the deadband isn't negative
        self.deadband = deadband

        # Initialize variables
        self.drivetrain = drivetrain
        self.kwargs = kwargs

        self.addRequirements(drivetrain) # Add requirements

    def initialize(self):
        pass # Nothing to do

    def isFinished(self) -> bool:
        return False  # never finishes

    def execute(self):
        """
        runs the function
        """
        self.drivetrain.drive(
            applyDeadband(self.forwardSpeed(), self.deadband),
            applyDeadband(self.leftSpeed(), self.deadband),
            applyDeadband(self.rotationSpeed(), self.deadband),
            **self.kwargs
        )

    def end(self, interrupted: bool):
        self.drivetrain.stop()  # stops the drivetrain when it's done
