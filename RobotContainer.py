import wpilib as wp
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
from commands2.button import JoystickButton, CommandPS4Controller

from subsystems.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand
from constants import Constants, SwerveConstants


class RobotContainer:
    swerve_subsystem = SwerveSubsystem()

    controller = CommandPS4Controller(Constants.pilot_controller_id)

    field_oriented = False

    right_stick_sets_angle = True

    def __init__(self) -> None:
        self.configure_button_bindings()

        self.swerve_subsystem.setDefaultCommand(
            SwerveCommand(
                self.swerve_subsystem,
                self.controller,
                self.get_right_stick_sets_angle,
                self.get_field_oriented,
            )
        )

    def get_right_stick_sets_angle(self) -> bool:
        return self.right_stick_sets_angle

    def teleopPeriodic(self) -> None:
        if self.controller.getSquareButtonPressed():
            self.toggle_field_oriented()

    def get_field_oriented(self) -> bool:
        return self.field_oriented

    def toggle_field_oriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def configure_button_bindings(self) -> None:
        self.controller.square().onTrue(InstantCommand(self.toggle_field_oriented))

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
            self.swerve_subsystem.get_pose,
            SwerveConstants.kDriveKinematics,
            x_pid,
            y_pid,
            theta_pid,
            self.swerve_subsystem.set_module_states_list,
            [self.swerve_subsystem],
        )

        return SequentialCommandGroup(
            InstantCommand(
                self.swerve_subsystem.reset_odometer, trajectory.initialPose
            ),
            swerve_command,
            InstantCommand(self.swerve_subsystem.stop),
        )
