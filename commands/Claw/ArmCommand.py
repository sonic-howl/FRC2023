import typing

from commands2 import CommandBase
from constants.ArmConstants import ArmConstants

from constants.GameConstants import GamePieceType
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class ArmCommand(CommandBase):
    def __init__(
        self,
        armAssembly: ArmAssemblySubsystem,
        getSelectedGamePiece: typing.Callable[[], GamePieceType],
        angleType: ArmConstants.AngleType,
    ) -> None:
        super().__init__()
        self.armAssembly = armAssembly
        self.addRequirements(self.armAssembly)

        self.getSelectedGamePiece = getSelectedGamePiece
        self.angleType = angleType

    def initialize(self) -> None:
        self.armAssembly.arm.stopHoldingPosition()
        self.armAssembly.claw.stopHoldingPosition()

    def execute(self) -> None:
        armAngle = ArmConstants.angles[self.getSelectedGamePiece()][self.angleType][
            ArmConstants.SubsystemType.kArm
        ]

        clawAngle = ArmConstants.angles[self.getSelectedGamePiece()][self.angleType][
            ArmConstants.SubsystemType.kClaw
        ]

        # TODO might have to add logic to avoid claw collisions using the arm's current angle.
        self.armAssembly.moveArmAndClawToAngles(armAngle, clawAngle)

    def end(self, interrupted: bool) -> None:
        self.armAssembly.arm.startHoldingPosition()
        self.armAssembly.claw.startHoldingPosition()

    def isFinished(self) -> bool:
        # return self.arm.atSetpoint()
        return False
