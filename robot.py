import wpilib
#import phoenix6
from phoenix6 import hardware, controls, configs, StatusCode
from wpimath.units import seconds
from robotContainer import RobotContainer

#note to self: use duty cycle control modes because they are simpler. 
# Might be a good idea to give figuring out voltage control stuff to Ryan 
# or Chris if we need to use it

class MyRobot(wpilib.TimedRobot):

    def __init__(self, period: float = 0.02) -> None:
        super().__init__(period)

    def robotInit(self) -> None:
        self.robotContainer = RobotContainer()
    
        '''self.motor1 = hardware.TalonFX(1)
        self.motor2 = hardware.TalonFX(3)
        
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        self.position_voltage = controls.PositionVoltage(0).with_slot(1)
        self.brake = controls.NeutralOut()
        
        self.joystick = wpilib.XboxController(0)
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_s = 0.1
        cfg.slot0.k_v = 0.12
        cfg.slot0.k_p = 0.11
        cfg.slot0.k_i = 0
        cfg.slot0.k_d = 0
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8
        cfg.slot1.k_p = 2.4
        cfg.slot1.k_i = 0
        cfg.slot1.k_d = 0.1

        self.motor1.configurator.apply(cfg)
        self.motor2.configurator.apply(cfg)

    def teleopInit(self) -> None:
        pass
'''

    def teleopInit(self) -> None:
        pass
    def teleopPeriodic(self) -> None:
       ''' joystick_value = self.joystick.getLeftY()
        if abs(joystick_value) < 0.1:
            joystick_value = 0

        joystick_X_value = self.joystick.getLeftX()
        if abs(joystick_X_value) < 0.1:
            joystick_X_value = 0
        
        desired_rotations_ps = joystick_value * 50
        self.motor1.set_control(self.velocity_voltage.with_velocity(desired_rotations_ps))

        desired_position = joystick_X_value * 10
        self.motor2.set_control(self.position_voltage.with_position(desired_position))'''
       
       pass