import typing
from commands2 import Command, Subsystem
from constants import ArmConstants
from wpimath.filter import SlewRateLimiter

from controllers.operator import OperatorController
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem


class MoveClawCommand(Command):
    """Allows operator to move the arm and claw manually using controller axes."""

    def __init__(
        self, armSubsystem: ArmAssemblySubsystem, controller: OperatorController
    ):
        super().__init__()

        self.armSubsystem = armSubsystem
        self.controller = controller

        self.armLimiter = SlewRateLimiter(ArmConstants.Arm.Manual.maxAnglePerSecond)
        self.clawLimiter = SlewRateLimiter(ArmConstants.Claw.Manual.maxAnglePerSecond)

    def initialize(self) -> None:
        pass

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.armSubsystem}

    def execute(self) -> None:
        self.armSubsystem.arm.armMotor.set(self.controller.getArmRotation())
        self.armSubsystem.claw.armMotor.set(self.controller.getClawRotation())
        # armOmega = (
        #     self.armLimiter.calculate(self.controller.getArmRotation())
        #     * ArmConstants.Arm.Manual.omegaScale
        # )
        # clawOmega = (
        #     self.clawLimiter.calculate(self.controller.getClawRotation())
        #     * ArmConstants.Claw.Manual.omegaScale
        # )

        # self.armSubsystem.arm.addAngle(armOmega)
        # self.armSubsystem.claw.addAngle(clawOmega)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return False
