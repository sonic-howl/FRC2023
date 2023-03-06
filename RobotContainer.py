from threading import Thread
from pathplannerlib import PathPlanner, PathConstraints
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
from commands2.button import JoystickButton, CommandPS4Controller

from subsystems.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand
from constants import Constants, SwerveConstants


from wpimath.filter._filter import SlewRateLimiter


def dz(x: float, dz: float = Constants.controller_deadzone):
    return x if abs(x) > dz else 0


class RobotContainer:
    swerve_subsystem = SwerveSubsystem()

    controller = CommandPS4Controller(Constants.pilot_controller_id)

    field_oriented = False

    right_stick_sets_angle = False

    def __init__(self) -> None:
        self.configure_button_bindings()

        # if self.controller.isConnected():
        #     self.swerve_subsystem.setDefaultCommand(
        #         SwerveCommand(
        #             self.swerve_subsystem,
        #             self.controller,
        #             self.get_right_stick_sets_angle,
        #             self.get_field_oriented,
        #         )
        #     )

        self.x_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.y_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.z_limiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxTurnAccelerationMetersPerSecond
        )

        self.rotate_to_angle_pid = PIDController(
            SwerveConstants.kPRobotTurn,
            SwerveConstants.kIRobotTurn,
            SwerveConstants.kDRobotTurn,
            period=Constants.period,
        )
        self.rotate_to_angle_pid.enableContinuousInput(-180, 180)
        self.rotate_to_angle_pid.setTolerance(5)  # degrees tolerance

    def get_right_stick_sets_angle(self) -> bool:
        return self.right_stick_sets_angle

    def get_angle(self):
        return self.swerve_subsystem.get_angle()

    def teleopPeriodic(self) -> None:
        if self.controller.getSquareButtonPressed():
            self.toggle_field_oriented()

        x = dz(self.controller.getRawAxis(0))
        y = -dz(self.controller.getRawAxis(1))

        # x = self.x_limiter.calculate(x)
        # y = self.y_limiter.calculate(y)

        chassis_speeds: ChassisSpeeds | None = None
        if self.get_right_stick_sets_angle():
            rx = dz(self.controller.getRawAxis(4))
            ry = dz(self.controller.getRawAxis(5))

            magnitude = abs(rx) + abs(ry)

            if magnitude > 0.25:
                rotation2d = Rotation2d(rx, ry)

                current_robot_angle = self.swerve_subsystem.get_angle()

                print(current_robot_angle, rotation2d.degrees())

                rotation_speed = self.rotate_to_angle_pid.calculate(
                    current_robot_angle, rotation2d.degrees()
                )

                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * SwerveConstants.kDriveMaxMetersPerSecond,
                    y * SwerveConstants.kDriveMaxMetersPerSecond,
                    rotation_speed,
                    self.swerve_subsystem.get_rotation2d(),
                )

            # print("SwerveCommand chassis_speeds: ", chassis_speeds)
        elif self.get_field_oriented():
            # z = dz(self.controller.getRightX())
            z = dz(self.controller.getRawAxis(4))
            z = self.z_limiter.calculate(z)
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * SwerveConstants.kDriveMaxMetersPerSecond,
                y * SwerveConstants.kDriveMaxMetersPerSecond,
                z,
                self.swerve_subsystem.get_rotation2d(),
            )
        else:
            z = dz(-self.controller.getRawAxis(4))
            magnitude = abs(x) + abs(y) + abs(z)
            if magnitude > 0.05:
                # z = self.z_limiter.calculate(z) # TODO
                chassis_speeds = ChassisSpeeds(
                    x * SwerveConstants.kDriveMaxMetersPerSecond,
                    y * SwerveConstants.kDriveMaxMetersPerSecond,
                    z,
                )

        if chassis_speeds:
            swerve_module_states = (
                SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
            )

            # if (
            #     abs(chassis_speeds.vx) > 0
            #     or abs(chassis_speeds.vy) > 0
            #     or abs(chassis_speeds.omega) > 0
            # ):
            #     Thread(
            #         target=print,
            #         args=[
            #             chassis_speeds,
            #             "SwerveCommand swerve_module_states: ",
            #             swerve_module_states,
            #         ],
            #     ).start()

            self.swerve_subsystem.set_module_states(swerve_module_states)
        else:
            self.swerve_subsystem.set_module_states((SwerveModuleState(),) * 4)

    # def autonomousPeriodic(self) -> None:

    def get_field_oriented(self) -> bool:
        return self.field_oriented

    def toggle_field_oriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def configure_button_bindings(self) -> None:
        if self.controller.isConnected():
            self.controller.square().onTrue(InstantCommand(self.toggle_field_oriented))

    def buildPPAutonomousCommand(self):
        path_group = PathPlanner.loadPathGroup(
            "FullPath",
            SwerveConstants.kDriveMaxMetersPerSecond,
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond,
        )

        # TODO

        for trajectory in path_group:
            print(trajectory.asWPILibTrajectory())

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
