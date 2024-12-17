import commands2
from drivetrain import DriveTrainSubSystem
from wpimath import geometry

class DefaultDriveCommand(commands2.Command):
    def __init__(self, driveTrainSubSystem: DriveTrainSubSystem):
        super().__init__()
        self.addRequirements(driveTrainSubSystem)
        self.driveTrain = driveTrainSubSystem

    def execute(self):
        self.driveTrain.joystickDrive(self.driveTrain.getJoystickInput())