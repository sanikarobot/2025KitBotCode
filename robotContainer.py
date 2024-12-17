from drivetrain import DriveTrainSubSystem
from defaultDriveCommand import DefaultDriveCommand
import commands2
import wpilib
import os
from wpimath import geometry, kinematics, estimator
import RobotConfig as config

class RobotContainer:
    def __init__(self) -> None:
        self.joystick = commands2.button.CommandJoystick(config.driveConstants.joystickConstants.USB_ID)
        self.driveTrain = DriveTrainSubSystem(self.joystick)
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))

    def checkJoystickInput(self, kInput: float):
        if kInput < 0.1:
            kInput = 0
        else:
            kInput = kInput/2
        return kInput