from ntcore import NetworkTableInstance
from controllers.operator import OperatorController
from commands2 import Command, Subsystem
from typing import Callable, Set
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from constants import LimelightConstants, SwerveConstants

from wpimath.trajectory import (
    TrajectoryConfig,
    TrajectoryGenerator,
    TrapezoidProfileRadians,
)

from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
)


class VisionTrackCommand(Command):
    
    def __init__(self, swerveSubsystem: SwerveSubsystem) -> None:
        super().__init__()
        ntInstance = NetworkTableInstance.getDefault()
        self.limelightTable = ntInstance.getTable("limelight")
        self.swerveSubsystem = swerveSubsystem
    
    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerveSubsystem}

    def initialize(self) -> None:
        print("VisionTrack initialized")
        # Switch to reflective tape detection pipeline
        self.limelightTable.getEntry("pipeline").setInteger(LimelightConstants.reflectiveTapePipelineID) # Not sure about these constant values

    def execute(self) -> None:
        trajectory_config = TrajectoryConfig(
        SwerveConstants.kDriveMaxMetersPerSecond,
        SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )
        trajectory_config.setKinematics(SwerveConstants.kDriveKinematics)

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            trajectory_config,
        )

                
    
    def end(self, interrupted: bool) -> None:
        print("VisionTrack Interupted")
        # Switch back to april tag detection
        self.limelightTable.getEntry("pipeline").setInteger(LimelightConstants.apriltagPipelineID)


    def isFinished(self) -> bool:
        return False
    








        # TODO make these constants in constants.py
        x_pid = PIDController(0.5, 0, 0, period=Constants.period)
        y_pid = PIDController(0.5, 0, 0, period=Constants.period)
        theta_pid = ProfiledPIDControllerRadians(
            0.5,
            0,
            0,
            TrapezoidProfileRadians.Constraints(3, 3),
            period=Constants.period,
        )
        theta_pid.enableContinuousInput(-180, 180)

        swerve_command = Swerve4ControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose,
            SwerveConstants.kDriveKinematics,
            x_pid,
            y_pid,
            theta_pid,
            self.swerveSubsystem.setModuleStates,
            [self.swerveSubsystem],
        )

        return SequentialCommandGroup(
            InstantCommand(self.swerveSubsystem.resetOdometer, trajectory.initialPose),
            swerve_command,
            InstantCommand(self.swerveSubsystem.stop),
        )