from constants.ArmConstants import ArmConstants

from .ArmCommand import ArmCommand


class L2GridCommand(ArmCommand):
    def execute(self) -> None:
        armAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kGridS2
        ][ArmConstants.SubsystemType.kArm]

        clawAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kGridS2
        ][ArmConstants.SubsystemType.kClaw]

        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.arm.setArmAndClawAngle(armAngle, clawAngle)
