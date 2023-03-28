import typing

from commands2 import Command, Subsystem

from constants.GameConstants import GamePieceType
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class ArmCommand(Command):
    def __init__(
        self,
        arm: ArmAssemblySubsystem,
        gamePieceType=GamePieceType.kEmpty,
    ) -> None:
        super().__init__()
        self.arm = arm
        self.gamePieceType = gamePieceType

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.arm}

    def isFinished(self) -> bool:
        return self.arm.atSetpoint()
