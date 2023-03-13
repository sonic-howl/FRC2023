import math
from commands2 import Command, Subsystem
from commands2.button import JoystickButton, CommandXboxController
from utils import print_async, sign
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from wpimath.filter._filter import SlewRateLimiter
from subsystems.SwerveSubsystem import SwerveSubsystem
from typing import Callable, Set

from constants import Constants, SwerveConstants


def dz(x: float, dz: float = Constants.controller_deadzone):
    return x if abs(x) > dz else 0


class SwerveCommand(Command):
    def __init__(
        self,
        swerve_subsystem: SwerveSubsystem,
        # get_x: Callable[[None], float],
        # get_y: Callable[[None], float],
        # get_z: Callable[[None], float],
        controller: CommandXboxController,
        get_right_stick_sets_angle: Callable[[], bool],
        get_field_oriented: Callable[[], bool],
    ) -> None:
        super().__init__()

        self.swerve_subsystem = swerve_subsystem

        # self.get_x = get_x
        # self.get_y = get_y
        # self.get_z = get_z

        self.controller = controller
        self.get_right_stick_sets_angle = get_right_stick_sets_angle
        self.get_field_oriented = get_field_oriented

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

    def getRequirements(self) -> Set[Subsystem]:
        return {self.swerve_subsystem}

    def initialize(self) -> None:
        print("SwerveCommand initialized")
        pass

    def execute(self) -> None:
        if not self.controller.isConnected():
            return

        # reversing x and y controller -> field axes. x is forwards, y is strafe
        # normally axis 0 is x.
        x = dz(-self.controller.getRawAxis(1))
        y = dz(-self.controller.getRawAxis(0))

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
            z = self.controller.getRawAxis(4)
            z = (dz(z) ** 2) * sign(z)
            z = self.z_limiter.calculate(z)
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x * SwerveConstants.kDriveMaxMetersPerSecond,
                y * SwerveConstants.kDriveMaxMetersPerSecond,
                z,
                self.swerve_subsystem.get_rotation2d(),
            )
        else:
            z = -self.controller.getRawAxis(4)
            z = (dz(z) ** 2) * sign(z)
            magnitude = abs(x) + abs(y) + abs(z)
            if magnitude > 0.05:
                # z = self.z_limiter.calculate(z) # TODO
                # chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                #     x * SwerveConstants.kDriveMaxMetersPerSecond,
                #     y * SwerveConstants.kDriveMaxMetersPerSecond,
                #     z,
                #     self.robot_angle_offset,
                # )
                chassis_speeds = ChassisSpeeds(
                    x * SwerveConstants.kDriveMaxMetersPerSecond,
                    y * SwerveConstants.kDriveMaxMetersPerSecond,
                    z,
                )

        if chassis_speeds:
            swerve_module_states = (
                SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
            )

            if (
                abs(chassis_speeds.vx) > 0
                or abs(chassis_speeds.vy) > 0
                or abs(chassis_speeds.omega) > 0
            ):
                print_async(
                    chassis_speeds,
                    "SwerveCommand swerve_module_states: ",
                    swerve_module_states[0],
                )
            # print_async(self.swerve_subsystem.get_pose())

            self.swerve_subsystem.set_module_states(swerve_module_states)
        else:
            self.swerve_subsystem.stop()

    def end(self, interrupted: bool) -> None:
        print("SwerveCommand ended")
        self.swerve_subsystem.stop()

    def isFinished(self) -> bool:
        return False
