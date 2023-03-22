from commands2.button import CommandPS4Controller
from constants import Constants


class PilotController:
    _controller = CommandPS4Controller(Constants.pilot_controller_id)

    def isConnected(self):
        return self._controller.isConnected()

    # reversing x and y controller -> field axes. x is forwards, y is strafe.
    def getForward(self):
        return -self._controller.getLeftY()

    def getStrafe(self):
        return self._controller.getLeftX()

    def getTurn(self):
        # return self._controller.getRightX()
        return self._controller.getRawAxis(2)

    def getSpeed(self):
        # return self._controller.getL2Axis()
        return (self._controller.getRawAxis(4) + 1) / 2

    def getRotateToAngle(self):
        return self._controller.getPOV()

    def fieldOrientedBtn(self):
        return self._controller.square()

    def resetGyroBtn(self):
        return self._controller.triangle()
