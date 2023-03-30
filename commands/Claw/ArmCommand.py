import typing

from commands2 import Command, Subsystem
from constants.ArmConstants import ArmConstants

from constants.GameConstants import GamePieceType
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class ArmCommand(Command):
    def __init__(
        self,
        arm: ArmAssemblySubsystem,
        getSelectedGamePiece: typing.Callable[[], GamePieceType],
        angleType: ArmConstants.AngleType,
    ) -> None:
        super().__init__()
        self.arm = arm
        self.getSelectedGamePiece = getSelectedGamePiece
        self.angleType = angleType

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.arm}
    
    def initialize(self) -> None:
        self.arm.stopHoldArmPosition()
        self.arm.stopHoldClawPosition()

    def execute(self) -> None:
        armAngle = ArmConstants.angles[self.getSelectedGamePiece()][self.angleType][
            ArmConstants.SubsystemType.kArm
        ]

        clawAngle = ArmConstants.angles[self.getSelectedGamePiece()][self.angleType][
            ArmConstants.SubsystemType.kClaw
        ]

        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.arm.setArmAndClawAngle(armAngle, clawAngle)

    def end(self, interrupted: bool) -> None:
        self.arm.holdArmPosition()
        self.arm.holdClawPosition()

    def isFinished(self) -> bool:
        # return self.arm.atSetpoint()
        return False
