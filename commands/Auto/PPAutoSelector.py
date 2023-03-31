import os
from commands.Auto.PPAutonomousCommand import PPAutonomousCommand
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem

from wpilib import SendableChooser, SmartDashboard


class PPAutonomousSelector:
    _PPPath = "/home/lvuser/py/deploy/pathplanner"

    def __init__(
        self,
        swerveSubsystem: SwerveSubsystem,
        armAssemblySubsystem: ArmAssemblySubsystem,
    ):
        self.swerveSubsystem = swerveSubsystem
        self.armAssemblySubsystem = armAssemblySubsystem

        self.chooser: SendableChooser | None = None

        self.makeChooser(self.readPathPlannerPaths())

    def readPathPlannerPaths(self):
        # I don't care about optimizing this
        files: list[str] = []
        for dirpath, dirnames, filenames in os.walk(self._PPPath):
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
                self.swerveSubsystem, self.armAssemblySubsystem, autoPaths[0]
            ),
        )

        for path in autoPaths[1:]:
            self.chooser.addOption(
                path,
                PPAutonomousCommand(
                    self.swerveSubsystem, self.armAssemblySubsystem, path
                ),
            )

        SmartDashboard.putData("Auto Selector", self.chooser)

    def getSelectedAutonomousCommand(self) -> PPAutonomousCommand:
        return self.chooser.getSelected()
