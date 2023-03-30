import rev

from commands2 import Subsystem
import wpilib

from controllers.operator import OperatorController
from constants.RobotConstants import LimitSwitchConstants 


class PickupSubsystem(Subsystem):
    def __init__(self, operatorController: OperatorController) -> None:
        super().__init__()

        self.controller = operatorController

        self.pickupMotor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        self.pickupMotor.setSmartCurrentLimit(15)

        self.cubeLimitSwitch = wpilib.DigitalInput(LimitSwitchConstants.cubeLimitSwitchID)

        self.limitSwitchHitChecked = False

    def isCubeLimitSwitchHit(self):
        return self.cubeLimitSwitch.get()

    def getLimitSwitchHitChecked(self):
        return self.limitSwitchHitChecked

    def setLimitSwitchHitChecked(self, checked=True):
        self.limitSwitchHitChecked = checked

    def set(self, speed: float):
        self.pickupMotor.set(speed)
