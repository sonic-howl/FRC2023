from commands2.button import CommandXboxController
from constants import Constants


class OperatorController:
    _controller = CommandXboxController(Constants.operator_controller_id)

    def isConnected(self):
        return self._controller.isConnected()

    def getTopGrid(self):
        return self._controller.getYButton()

    def getMiddleGrid(self):
        return self._controller.getBButton()

    def getBottomGrid(self):
        return self._controller.getAButton()

    def getFloorPickupCube(self):
        return self._controller.getPOV() == 270  # left d-pad

    def getFloorPickupCone(self):
        return self._controller.getPOV() == 90  # right d-pad

    def getStowClaw(self):
        return self._controller.getPOV() == 0  # up d-pad

    # def getClawX(self):
    #     """
    #     Returns the X axis of the right joystick.
    #     This will control the position of the arm/claw in the X direction (forward).
    #     """
    #     return self._controller.getLeftTriggerAxis()

    # def getClawY(self):
    #     """
    #     Returns the Y axis of the right joystick.
    #     This will control the position of the arm/claw in the Y direction (up and down).
    #     """
    #     return self._controller.getRightTriggerAxis()

    def getClawRotation(self):
        """
        Returns the left Y axis value.
        This will control the rotation of the claw.
        """
        return -self._controller.getLeftY()

    def getArmRotation(self):
        """
        Returns the right Y axis value.
        This will control the rotation of the claw.
        """
        return -self._controller.getRightY()
