import os
from commands.Auto.PPAutonomousCommand import PPAutonomousCommand
from constants.RobotConstants import RobotConstants
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem
from subsystems.Pickup.PickupSubsystem import PickupSubsystem
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem

from wpilib import SendableChooser, SmartDashboard


class PPAutonomousSelector:
    def __init__(
        self,
        swerveSubsystem: SwerveSubsystem,
        armAssemblySubsystem: ArmAssemblySubsystem,
        pickupSubsystem: PickupSubsystem,
    ):
        self.swerveSubsystem = swerveSubsystem
        self.armAssemblySubsystem = armAssemblySubsystem
        self.pickupSubsystem = pickupSubsystem

        # best variable name ever
        self._PPPathsPath = (
            "/home/lvuser/py/deploy/pathplanner"
            if not RobotConstants.isSimulation
            else f"{os.getcwd()}/deploy/pathplanner"
        )

        self.chooser: SendableChooser | None = None

        self.makeChooser(self.readPathPlannerPaths())

    def readPathPlannerPaths(self):
        # I don't care about optimizing this
        files: list[str] = []
        for dirpath, dirnames, filenames in os.walk(self._PPPathsPath):
            files.extend(filenames)
            break

        filteredFiles: list[str] = []
        for file in files:
            if file.endswith(".path"):
                filteredFiles.append(file.replace(".path", ""))

        return filteredFiles

    def makeChooser(self, autoPaths: list[str]):
        self.chooser = SendableChooser()

        self.chooser.setDefaultOption(
            autoPaths[0],
            PPAutonomousCommand(
                self.swerveSubsystem,
                self.armAssemblySubsystem,
                self.pickupSubsystem,
                autoPaths[0],
            ),
        )

        for path in autoPaths[1:]:
            self.chooser.addOption(
                path,
                PPAutonomousCommand(
                    self.swerveSubsystem,
                    self.armAssemblySubsystem,
                    self.pickupSubsystem,
                    path,
                ),
            )

        SmartDashboard.putData("Auto Selector", self.chooser)

    def getSelectedAutonomousCommand(self) -> PPAutonomousCommand:
        return self.chooser.getSelected()
