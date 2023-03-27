from .ArmCommand import ArmCommand
from constants import ArmConstants


class StowCommand(ArmCommand):
    def initialize(self) -> None:
        armAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kStow
        ][ArmConstants.SubsystemType.kArm]

        clawAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kStow
        ][ArmConstants.SubsystemType.kClaw]

        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.arm.setArmAndClawAngle(armAngle, clawAngle)
