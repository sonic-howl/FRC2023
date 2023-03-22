import typing
from commands2 import Command, Subsystem, InstantCommand
from constants import ArmConstants
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class StowCommand(InstantCommand):
    def __init__(
        self, arm: ArmAssemblySubsystem, gamePieceType=ArmConstants.GamePieceType.kEmpty
    ) -> None:
        super().__init__()
        self.arm = arm
        self.gamePieceType = gamePieceType

    def execute(self) -> None:
        self.arm.stow(self.gamePieceType)
        return super().execute()


# class StowCommand(Command):
#     def __init__(
#         self, arm: ArmAssemblySubsystem, gamePieceType=ArmConstants.GamePieceType.kEmpty
#     ) -> None:
#         super().__init__()

#         self.arm = arm
#         self.gamePieceType = gamePieceType

#         self.finished = False

#     def getRequirements(self) -> typing.Set[Subsystem]:
#         return {self.arm}

#     def initialize(self) -> None:
#         pass

#     def execute(self) -> None:
#         # this only needs to execute once since it sets a reference angle to the spark max.
#         self.arm.stow(self.gamePieceType)
#         self.finished = True

#     def end(self, interrupted: bool) -> None:
#         pass

#     def isFinished(self) -> bool:
#         return self.finished
