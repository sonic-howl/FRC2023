from .ArmCommand import ArmCommand
from constants import ArmConstants


class FloorPickupCommand(ArmCommand):
    def initialize(self) -> None:
        armAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kFloor
        ][ArmConstants.SubsystemType.kArm]

        clawAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kFloor
        ][ArmConstants.SubsystemType.kClaw]

        self.arm.setArmAndClawAngle(armAngle, clawAngle)