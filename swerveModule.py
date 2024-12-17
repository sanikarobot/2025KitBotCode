import phoenix6
from wpimath import kinematics, geometry
import wpilib
import math
from phoenix6 import hardware, controls, signals
import RobotConfig as rc

class SwerveModule:

    def __init__(self, driveID: int, turnID: int, coderID: int) -> None:
        
        self.desiredState = kinematics.SwerveModuleState(0, geometry.Rotation2d())
        
        self.driveMotor = hardware.TalonFX(driveID)
        self.turnMotor = hardware.TalonFX(turnID)
        self.canCoder = hardware.CANcoder(coderID)

        self.velocity = controls.VelocityVoltage(0).with_slot(0)
        self.position = controls.PositionVoltage(0).with_slot(0)
        self.brake = controls.NeutralOut()

        canCoderConfigs = phoenix6.configs.CANcoderConfiguration()

        canCoderConfigs.magnet_sensor.absolute_sensor_range = signals.AbsoluteSensorRangeValue.UNSIGNED_0_TO1
        
        self.canCoder.configurator.apply(canCoderConfigs)

        driveMotorConfigs = phoenix6.configs.TalonFXConfiguration()

        driveMotorConfigs.slot0.k_p = 0.1
        driveMotorConfigs.slot0.k_i = 0.0001
        driveMotorConfigs.slot0.k_d = 0.0001
        driveMotorConfigs.slot0.k_s = 0.1
        driveMotorConfigs.slot0.k_v = 0.12
        driveMotorConfigs.voltage.peak_forward_voltage = 8
        driveMotorConfigs.voltage.peak_reverse_voltage = -8
        driveMotorConfigs.current_limits.supply_current_limit = 12 #find a real number for this
        driveMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        driveMotorConfigs.feedback.sensor_to_mechanism_ratio = 12 #check swerve stuff to get a real number

        self.driveMotor.configurator.apply(driveMotorConfigs)

        turnMotorConfigs = phoenix6.configs.TalonFXConfiguration()

        turnMotorConfigs.slot0.k_p = 0.0005
        turnMotorConfigs.slot0.k_i = 0.0005
        turnMotorConfigs.slot0.k_d = 0.0
        turnMotorConfigs.voltage.peak_forward_voltage = 8
        turnMotorConfigs.voltage.peak_reverse_voltage = -8
        turnMotorConfigs.feedback.feedback_remote_sensor_id = coderID
        turnMotorConfigs.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        turnMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        turnMotorConfigs.feedback.sensor_to_mechanism_ratio = 12 #check swerve drive specs to get a real number

        turnMotorConfigs.current_limits.supply_current_limit = 12 #find a real number for this

        self.turnMotor.configurator.apply(turnMotorConfigs)

        self.turnMotor.set_position(0)
        self.desiredState.angle = geometry.Rotation2d(self.getWheelAngleRadians()) 

    def getWheelAngleRadians(self):
        value = self.turnMotor.get_position().value / 360
        return math.radians(value)  
    
    def getTurnWheelState(self)-> geometry.Rotation2d:
        return geometry.Rotation2d(self.getWheelAngleRadians())
    
    def getDriveState(self):
        #not sure about if this will return as the correct data type
        return self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi

    def getState(self)-> kinematics.SwerveModuleState:
        return kinematics.SwerveModuleState(self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def getPosition(self)-> kinematics.SwerveModulePosition:
        return kinematics.SwerveModulePosition(self.driveMotor.get_position().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def setState(self, desiredState: kinematics.SwerveModuleState)-> None:
        optimizedDesiredState = kinematics.SwerveModuleState.optimize(desiredState, geometry.Rotation2d(self.turnMotor.get_position().value))
        driveMotorVelocity = optimizedDesiredState.speed / (rc.driveConstants.wheelDiameter * math.pi)
        turnMotorPosition = optimizedDesiredState.angle / math.tau
        self.driveMotor.set_control(self.velocity(driveMotorVelocity))
        self.turnMotor.set_control(self.position(turnMotorPosition))
        self.desiredState = desiredState

    def setNuetral(self)-> None:
        self.turnMotor.set_control(self.brake)
        self.turnMotor.set_control(self.brake)

    def stop(self)-> None:
        self.driveMotor.set_control(self.velocity(0))
        self.turnMotor.set_control(self.velocity(0))

    
