from threading import Thread
from time import sleep
from typing import Callable
from constants.GeneralConstants import POVAngles

from commands2.button import CommandXboxController, POVButton
from wpimath.filter import Debouncer

from constants.RobotConstants import RobotConstants
from utils.utils import dz


class OperatorController:
    _controller = CommandXboxController(RobotConstants.operator_controller_id)

    def isConnected(self):
        return self._controller.isConnected()

    def onceConnected(self, cb: Callable[[], None], checkInterval=1):
        def checkConnection():
            while not self.isConnected():
                sleep(checkInterval)
            print("Operator controller connected!")
            cb()

        Thread(target=checkConnection).start()

    def getL3Grid(self):
        return self._controller.Y()

    def getL2Grid(self):
        return self._controller.B()

    def getL1Grid(self):
        return self._controller.A()

    def getFloorPickup(self):
        return POVButton(self._controller, POVAngles.DOWN)

    def getStowClaw(self):
        return POVButton(self._controller, POVAngles.LEFT)

    def getUpperFeedStation(self):
        return POVButton(self._controller, POVAngles.UP)

    def getConeSelected(self):
        return self._controller.leftBumper().and_(self._controller.rightBumper().not_())

    def getCubeSelected(self):
        return self._controller.rightBumper().and_(self._controller.leftBumper().not_())

    def getEmptySelected(self):
        # return (
        #     self._controller.leftBumper()
        #     .and_(self._controller.rightBumper())
        #     .debounce(0.25, Debouncer.DebounceType.kFalling)
        # )  # TODO try this
        return POVButton(self._controller, POVAngles.RIGHT)

    def getClawRotation(self):
        return dz(self._controller.getLeftY())

    def getArmRotation(self):
        return -dz(self._controller.getRightY())

    def getResetArmAndClawPosition(self):
        return self._controller.X()

    def getPickupIntakeSpeed(self):
        return self._controller.getRightTriggerAxis()

    def getPickupReleaseSpeed(self):
        return -self._controller.getLeftTriggerAxis()
