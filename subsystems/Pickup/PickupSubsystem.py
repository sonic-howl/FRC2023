import rev

from commands2 import Subsystem
import wpilib

from controllers.operator import OperatorController


class PickupSubsystem(Subsystem):
    def __init__(self, operatorController: OperatorController) -> None:
        super().__init__()

        self.controller = operatorController

        self.pickupMotor = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        self.pickupMotor.setSmartCurrentLimit(15)

        self.limitSwitchHitChecked = False

    def getLimitSwitchHitChecked(self):
        return self.limitSwitchHitChecked

    def setLimitSwitchHitChecked(self, checked=True):
        self.limitSwitchHitChecked = checked

    def set(self, speed: float):
        self.pickupMotor.set(speed)
