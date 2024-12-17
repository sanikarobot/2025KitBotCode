from wpimath import geometry, kinematics
import wpilib
import navx
from swerveModule import SwerveModule
from wpilib import DriverStation
from wpimath import controller, trajectory, estimator
import wpimath
from math import hypot, radians, pi, atan2
from typing import List
import commands2
import RobotConfig as rc

class DriveTrainSubSystem(commands2.Subsystem):

    def __init__(self, joystick: commands2.button.CommandJoystick) -> None:

        self.stateStdDevs = 0.1, 0.1, 0.1
        self.visionMeasurementsStdDevs = 0., 0.9, 0.9

        self.joystick = joystick
        self.navx = navx.AHRS.create_spi(update_rate_hz=60)

        self.kMaxSpeed = rc.driveConstants.RobotSpeeds.maxSpeed
        self.kMaxAngularVelocity = rc.driveConstants.RobotSpeeds.maxSpeed /hypot(rc.robotDimensions.trackWidth / 2, rc.robotDimensions.wheelBase / 2)
        self.wheelBase = rc.robotDimensions.wheelBase
        self.trackWidth = rc.robotDimensions.trackWidth

        self.frontLeft = SwerveModule(rc.SwerveModules.frontLeft.driveMotorID, rc.SwerveModules.frontLeft.turnMotorID, rc.SwerveModules.frontLeft.CANCoderID)
        self.frontRight = SwerveModule(rc.SwerveModules.frontRight.driveMotorID, rc.SwerveModules.frontRight.turnMotorID, rc.SwerveModules.frontRight.CANCoderID)
        self.rearLeft = SwerveModule(rc.SwerveModules.rearLeft.driveMotorID, rc.SwerveModules.rearLeft.turnMotorID, rc.SwerveModules.rearLeft.CANCoderID)
        self.rearRight = SwerveModule(rc.SwerveModules.rearRight.driveMotorID, rc.SwerveModules.rearRight.turnMotorID, rc.SwerveModules.rearRight.CANCoderID)

        teleopConstants = rc.driveConstants.poseConstants

        rotationConstants = rc.driveConstants.ThetaPIDConstants.translationPIDConstants
        self.rotationPID = controller.PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, rotationConstants.period)
        self.rotationPID.enableContinuousInput(-pi, pi)

        self.poseTolerance = geometry.Pose2d(geometry.Translation2d(x=teleopConstants.xPoseToleranceMeters, y=teleopConstants.yPoseToleranceMeters), geometry.Rotation2d(teleopConstants.thetaPoseToleranceRadians))
        self.alliance = wpilib.DriverStation.Alliance.kBlue

        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2)))
        self.poseEstimatior = estimator.SwerveDrive4PoseEstimator(kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2))), 
                                                                  self.getNavxRotation2d(), self.getSwerveModulePositions(), geometry.Pose2d(0, 0, geometry.Rotation2d()), self.stateStdDevs, self.visionMeasurementsStdDevs)
        
        self.field = wpilib.Field2d
        #wpilib.SmartDashboard.putData("Field: ", self.field)
        
    def getNavxRotation2d(self)-> geometry.Rotation2d:
        return self.navx.getRotation2d()
    
    def getPose(self)-> geometry.Pose2d:
        return self.poseEstimatior.getEstimatedPosition()
    
    def getSwerveModulePositions(self):
        return self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()
    
    def resetPose(self, poseToSet: geometry.Pose2d)-> None:
        self.poseEstimatior.resetPosition(self.getNavxRotation2d(), self.getSwerveModulePositions(), poseToSet)

    def resetFieldOrient(self)-> None:
        self.navx.reset()

    def getJoystickInput(self)-> tuple[float]:
        constants = rc.driveConstants.joystickConstants
        return(-wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband),
               -wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband),
               -wpimath.applyDeadband(self.joystick.getZ(), constants.theataDeadband))
    
    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient = True)-> None:
        if fieldOrient:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed), self.poseEstimatior.getEstimatedPosition().rotation())
        else:
            SwerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))

        self.frontLeft.setState(SwerveModuleStates[0])
        self.frontRight.setState(SwerveModuleStates[1])
        self.rearLeft.setState(SwerveModuleStates[2])
        self.rearRight.setState(SwerveModuleStates[3])
    
    def joystickDrive(self, inputs: tuple[float])-> None:
        xSpeed, ySpeed, zSpeed, = (inputs[0] * self.kMaxSpeed,
                                   inputs[1] * self.kMaxSpeed,
                                   input[2] * self.kMaxAngularVelocity * rc.driveConstants.RobotSpeeds.manualRotationSpeedFactor)
        self.setSwerveStates(xSpeed, ySpeed, zSpeed, self.poseEstimatior.getEstimatedPosition())

    def stationary(self)-> None:
        self.frontLeft.stop()
        self.frontRight.stop()
        self.rearLeft.stop()
        self.rearRight.stop()

    def coast(self)-> None:
        self.frontLeft.setNuetral()
        self.frontRight.setNuetral()
        self.rearLeft.setNuetral()
        self.rearRight.setNuetral()

    def getRobotRelativeChassisSpeeds(self):
        states = (self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition())
        return self.KINEMATICS.toChassisSpeeds(states)
    
    def getCurrentPose(self)-> geometry.Pose2d:
        return self.poseEstimatior.getEstimatedPosition()
    
    def periodic(self) -> None:
        currentPose = self.poseEstimatior.update(self.getNavxRotation2d, self.getSwerveModulePositions())
        self.field.setRobotPose(currentPose)

    


