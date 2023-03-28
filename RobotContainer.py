from LightStrip import LightStrip
from commands.Claw.MoveClawCommand import MoveClawCommand
from controllers.operator import OperatorController
from controllers.pilot import PilotController

from photonvision import PhotonCamera
from commands.Auto.SwerveAutoCommand import SwerveAutoCommand
from subsystems.Arm.ArmAssemblySubsystem import ArmAssemblySubsystem
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
    InstantCommand,
    Swerve4ControllerCommand,
    SequentialCommandGroup,
)

from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand
from constants import ArmConstants, Constants, SwerveConstants


class RobotContainer:
    swerveSubsystem = SwerveSubsystem()
    armAssemblySubsystem = ArmAssemblySubsystem()

    pilotController = PilotController()
    operatorController = OperatorController()

    field_oriented = True

    photon_camera = PhotonCamera("photonvision")

    selectedGamePiece = ArmConstants.GamePieceType.kEmpty

    def __init__(self) -> None:
        self.setupSwerve()
        self.setupArm()

        # self.light_strip = LightStrip(Constants.light_strip_pwm_port)
        # self.light_strip.setRainbowSlow()

    @staticmethod
    def getSelectedGamePiece():
        return RobotContainer.selectedGamePiece

    @staticmethod
    def setSelectedGamePiece(gamePiece: ArmConstants.GamePieceType):
        RobotContainer.selectedGamePiece = gamePiece

    def get_angle(self):
        return self.swerveSubsystem.getAngle()

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

        self.swerveSubsystem.setDefaultCommand(
            SwerveCommand(
                self.swerveSubsystem,
                self.pilotController,
                self.getFieldOriented
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

        self.swerveAutoCommand = SwerveAutoCommand(
            self.swerveSubsystem, self.armAssemblySubsystem
        )

    def getFieldOriented(self) -> bool:
        return self.field_oriented

    def toggleFieldOriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def setupArm(self):
        self.configureArmButtonBindings()

    def setupArmTeleopInit(self):
        self.armAssemblySubsystem.setDefaultCommand(
            MoveClawCommand(self.armAssemblySubsystem, self.operatorController)
        )

    def configureArmButtonBindings(self) -> None:
        if self.operatorController.isConnected():
            self.operatorController.getZeroEncoderPosition().onTrue(
                InstantCommand(self.armAssemblySubsystem.resetArm)
            )

    def configureSwerveButtonBindings(self) -> None:
        if self.pilotController.isConnected():
            self.pilotController.fieldOrientedBtn().onTrue(
                InstantCommand(self.toggleFieldOriented)
            )
            self.pilotController.resetGyroBtn().onTrue(
                InstantCommand(self.swerveSubsystem.resetGyro)
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
