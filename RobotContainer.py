from typing import List
from LightStrip import LightStrip
from controllers.pilot import PilotController

from photonvision import PhotonCamera, LEDMode, RobotPoseEstimator
from wpilib import Field2d, SmartDashboard
from commands.SwerveAutoCommand import SwerveAutoCommand
from pathplannerlib import PathPlanner, PathConstraints, PathPlannerTrajectory
from pathplannerlib._pathplannerlib.controllers import PPHolonomicDriveController
from utils.utils import printAsync, sign
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
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
from commands2 import (
    RunCommand,
    InstantCommand,
    Swerve4ControllerCommand,
    SequentialCommandGroup,
    ParallelCommandGroup,
)
from commands2.button import JoystickButton, CommandXboxController

from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand
from constants import Constants, SwerveConstants


from wpimath.filter._filter import SlewRateLimiter


class RobotContainer:
    swerve_subsystem = SwerveSubsystem()

    controller = PilotController()

    field_oriented = True

    photon_camera = PhotonCamera("photonvision")

    def __init__(self) -> None:
        self.setupSwerve()

        self.light_strip = LightStrip(Constants.light_strip_pwm_port)
        self.light_strip.setRainbowSlow()

    def get_angle(self):
        return self.swerve_subsystem.getAngle()

    def vision_track(self):
        if self.photon_camera.hasTargets():
            target = self.photon_camera.getLatestResult().getBestTarget()
            transform_to_target = target.getBestCameraToTarget()
            # TODO

    # def robotPeriodic(self):
    #     self.light_strip.update()

    #     self.swerve_subsystem.periodic()

    #     if self.controller.isConnected():
    #         if self.controller.getAButton():
    #             self.vision_track()

    # def autonomousInit(self):
    #     self.swerve_auto_command.initialize()

    # def autonomousPeriodic(self) -> None:
    #     if self.swerve_auto_command.isFinished():
    #         self.swerve_subsystem.stop()
    #     else:
    #         self.swerve_auto_command.execute()

    def setupSwerve(self):
        self.configureSwerveButtonBindings()

        self.swerve_subsystem.setDefaultCommand(
            SwerveCommand(
                self.swerve_subsystem,
                self.controller,
                self.getFieldOriented,
            )
        )

        self.rotate_to_angle_pid = PIDController(
            SwerveConstants.kPRobotTurn,
            SwerveConstants.kIRobotTurn,
            SwerveConstants.kDRobotTurn,
            period=Constants.period,
        )
        self.rotate_to_angle_pid.enableContinuousInput(-180, 180)
        self.rotate_to_angle_pid.setTolerance(3)  # degrees tolerance

        self.swerve_auto_command = SwerveAutoCommand(self.swerve_subsystem)

    def getFieldOriented(self) -> bool:
        return self.field_oriented

    def toggleFieldOriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def configureSwerveButtonBindings(self) -> None:
        self.controller.fieldOrientedBtn().onTrue(
            InstantCommand(self.toggleFieldOriented)
        )
        self.controller.resetGyroBtn().onTrue(
            InstantCommand(self.swerve_subsystem.reset_gyro())
        )

    def getAutonomousCommand(self):
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
            self.swerve_subsystem.getPose,
            SwerveConstants.kDriveKinematics,
            x_pid,
            y_pid,
            theta_pid,
            self.swerve_subsystem.setModuleStates,
            [self.swerve_subsystem],
        )

        return SequentialCommandGroup(
            InstantCommand(self.swerve_subsystem.resetOdometer, trajectory.initialPose),
            swerve_command,
            InstantCommand(self.swerve_subsystem.stop),
        )
