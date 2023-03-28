from ntcore import NetworkTableInstance
from controllers.operator import OperatorController
from commands2 import Command, Subsystem
from typing import Callable, Set
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from controllers.pilot import PilotController
from wpimath.estimator import SwerveDrive4PoseEstimator


class CalibratePoseCommand(Command):
    
    def __init__(self, swerveSubsystem: SwerveSubsystem) -> None:
        super().__init__()
        ntInstance = NetworkTableInstance.getDefault()
        self.limelightTable = ntInstance.getTable("limelight")
        self.swerveSubsystem = swerveSubsystem
    
    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerveSubsystem}

    def initialize(self) -> None:
        print("Vision Command initialized")

    def execute(self) -> None:
        if self.limelightTable.getIntegerTopic("tv"):
            self.newPose = self.limelightTable.getDoubleArrayTopic("botpose")

                
    
    def end(self, interrupted: bool) -> None:
        print("Pose Calibrated")

    def isFinished(self) -> bool:
        return False