import math
from typing import Type
import rev

from wpimath.controller import ArmFeedforward
from commands2 import SubsystemBase

from constants import ArmConstants, Constants
from .ArmSubsystem import ArmSubsystem


class ArmAssemblySubsystem(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.arm = ArmSubsystem(ArmConstants.Arm)
        self.claw = ArmSubsystem(ArmConstants.Claw)

    def getClawAngle(self):
        """Returns the angle of the claw"""
        return self.claw.getAngle()

    def getRealClawAngle(self):
        """Returns the angle of the claw added to the angle of the arm to get the total angle of the claw"""
        return (self.getClawAngle() + self.getArmAngle()) % 360

    def setClawAngle(self, angle: float):
        """Sets the angle of the claw to the given angle"""
        self.claw.setAngle(angle)

    def getArmAngle(self):
        """Returns the angle of the arm"""
        return self.arm.getAngle()

    def setArmAngle(self, angle: float):
        """Sets the angle of the arm to the given angle"""
        self.arm.setAngle(angle)

    def setArmAndClawAngle(self, armAngle: float, clawAngle: float):
        """Sets the angle of the arm and claw to the given angles"""
        self.arm.setAngle(armAngle)
        self.claw.setAngle(clawAngle)

    def atSetpoint(self):
        """Returns true if the arm and claw are at their setpoints"""
        return self.arm.atSetpoint() and self.claw.atSetpoint()

    def resetArm(self):
        self.arm.setPosition(self.arm.initialPosition)

    def resetClaw(self):
        self.claw.setPosition(self.claw.initialPosition)
