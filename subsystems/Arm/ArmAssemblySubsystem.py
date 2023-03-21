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

    # def getArmAngle(self):
    #     # return self.arm_motor.getSelectedSensorPosition() * 360 / 4096
    #     # return self.arm_motor.getSelectedSensorPosition() * 360 / 4096
    #     # return Conversions.falconToDegrees(
    #     #     self.armMotor.getSelectedSensorPosition(), ArmConstants.kArmReduction
    #     # )
    #     return self.armEncoder.getPosition() % 360

    # def getRawClawAngle(self):
    #     return self.clawEncoder.getPosition() % 360

    def getClawAngle(self):
        """Returns the angle of the claw added to the angle of the arm to get the total angle of the claw"""
        return (self.claw.getAngle() - self.arm.getAngle()) % 360

    def stow(self, gamePieceType=ArmConstants.GamePieceType.kEmpty):
        self.arm.setAngle(
            ArmConstants.angles[gamePieceType][ArmConstants.AngleType.kStow][
                ArmConstants.SubsystemType.kArm
            ]
        )
        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.claw.setAngle(
            ArmConstants.angles[gamePieceType][ArmConstants.AngleType.kStow][
                ArmConstants.SubsystemType.kClaw
            ]
        )
