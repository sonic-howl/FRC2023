import typing
from commands2 import Command, Subsystem
from constants import GamePieceType
from controllers.operator import OperatorController

from subsystems.Pickup.PickupSubsystem import PickupSubsystem


class PickupCommand(Command):
    def __init__(
        self,
        pickup: PickupSubsystem,
        controller: OperatorController,
        getGamePieceSelected: typing.Callable[[], GamePieceType],
    ) -> None:
        super().__init__()

        self.controller = controller
        self.pickup = pickup
        self.getGamePieceSelected = getGamePieceSelected

        self.stopPickupOnLimitSwitchWhileTriggerHeld = False

    def getRequirements(self) -> typing.Set[Subsystem]:
        return {self.pickup}

    def execute(self) -> None:
        # read axes and set speed depending on the game piece selected.
        # if the limit switch is hit while the cube is selected, stop motion
        relSpeed = self.controller.getPickupReleaseSpeed()
        intakeSpeed = self.controller.getPickupReleaseSpeed()
        speed = relSpeed + intakeSpeed

        if self.stopPickupOnLimitSwitchWhileTriggerHeld:
            if speed == 0:
                self.stopPickupOnLimitSwitchWhileTriggerHeld = False
            self.pickup.set(0)
            return

        match self.getGamePieceSelected():
            case GamePieceType.kEmpty:
                speed = 0
            case GamePieceType.kCube:
                if (
                    self.pickup.isCubeLimitSwitchHit()
                    and not self.pickup.getLimitSwitchHitChecked()
                ):
                    speed = 0
                    self.stopPickupOnLimitSwitchWhileTriggerHeld = True
                    self.pickup.setLimitSwitchHitChecked(True)
                else:
                    self.pickup.setLimitSwitchHitChecked(False)
            case GamePieceType.kCone:
                speed = -speed

        self.pickup.set(speed)

    def isFinished(self) -> bool:
        return False
