from commands2 import SubsystemBase

from constants import ArmConstants
from controllers.operator import OperatorController
from .ArmSubsystem import ArmSubsystem


class ArmAssemblySubsystem(SubsystemBase):
    def __init__(self, operatorController: OperatorController) -> None:
        super().__init__()

        self.operatorController = operatorController

        self.arm = ArmSubsystem(ArmConstants.Arm)
        self.claw = ArmSubsystem(ArmConstants.Claw)

    def periodic(self) -> None:
        if self.operatorController.isConnected():
            # if there is manual control, cancel the current command to run the default command
            if (
                self.operatorController.getArmRotation() != 0
                or self.operatorController.getClawRotation() != 0
            ):
                # let manual control override the current command
                currentCommand = self.getCurrentCommand()
                if self.getDefaultCommand() != currentCommand:
                    currentCommand.cancel()

    def getClawAngle(self):
        """Returns the angle of the claw"""
        return self.claw.getAngle()

    def getRealClawAngle(self):
        """Returns the angle of the claw added to the angle of the arm to get the total angle of the claw"""
        return (self.getClawAngle() + self.getArmAngle()) % 360

    def setClawAngle(self, angle: float):
        """Sets the angle of the claw to the given angle"""
        self.claw.setAngle(angle)

    def getArmAngle(self):
        """Returns the angle of the arm"""
        return self.arm.getAngle()

    def setArmAngle(self, angle: float):
        """Sets the angle of the arm to the given angle"""
        self.arm.setAngle(angle)

    def setArmAndClawAngle(self, armAngle: float, clawAngle: float):
        """Sets the angle of the arm and claw to the given angles"""
        self.arm.setAngle(armAngle)
        self.claw.setAngle(clawAngle)

    def atSetpoint(self):
        """Returns true if the arm and claw are at their setpoints"""
        return self.arm.atSetpoint() and self.claw.atSetpoint()

    def resetArm(self):
        self.arm.setPosition(self.arm.initialPosition)

    def resetClaw(self):
        self.claw.setPosition(self.claw.initialPosition)
