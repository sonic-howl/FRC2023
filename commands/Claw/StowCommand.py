import typing
from commands2 import Command, Subsystem
from constants import ArmConstants
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class StowCommand(Command):
    def __init__(
        self,
        arm: ArmAssemblySubsystem,
        gamePieceType=ArmConstants.GamePieceType.kEmpty,
    ) -> None:
        super().__init__()
        self.arm = arm
        self.gamePieceType = gamePieceType

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.arm}

    def initialize(self) -> None:
        print("StowCommand: initialize()")

        armAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kStow
        ][ArmConstants.SubsystemType.kArm]

        clawAngle = ArmConstants.angles[self.gamePieceType][
            ArmConstants.AngleType.kStow
        ][ArmConstants.SubsystemType.kClaw]

        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.arm.setArmAndClawAngle(armAngle, clawAngle)

    def isFinished(self) -> bool:
        return self.arm.atSetpoint()
