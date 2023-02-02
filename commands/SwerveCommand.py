import math
from commands2 import Command, Subsystem
from commands2.button import JoystickButton, CommandPS4Controller
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
        controller: CommandPS4Controller,
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
        # x = dz(self.get_x())
        # y = dz(self.get_y())
        # z = dz(self.get_z())

        x = dz(self.controller.getLeftX())
        y = dz(self.controller.getLeftY())

        x = self.x_limiter.calculate(x)
        y = self.y_limiter.calculate(y)

        chassis_speeds: ChassisSpeeds
        if self.get_right_stick_sets_angle():
            rx = dz(self.controller.getRightX())
            ry = dz(self.controller.getRightY())
            rotation2d = Rotation2d(rx, ry)

            current_robot_angle = self.swerve_subsystem.get_angle()

            rotation_speed = self.rotate_to_angle_pid.calculate(
                current_robot_angle, rotation2d.degrees()
            )

            chassis_speeds = ChassisSpeeds(x, y, rotation_speed)

            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassis_speeds, self.swerve_subsystem.get_rotation2d()
            )

            # print("SwerveCommand chassis_speeds: ", chassis_speeds)
        elif self.get_field_oriented():
            z = dz(self.controller.getRightX())
            z = self.z_limiter.calculate(z)
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassis_speeds, self.swerve_subsystem.get_rotation2d()
            )
        else:
            z = dz(self.controller.getRightX())
            z = self.z_limiter.calculate(z)
            chassis_speeds = ChassisSpeeds(x, y, z)

        swerve_module_states = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            chassis_speeds
        )

        # print("SwerveCommand swerve_module_states: ", swerve_module_states)

        self.swerve_subsystem.set_module_states(swerve_module_states)

    def end(self, interrupted: bool) -> None:
        print("SwerveCommand ended")
        self.swerve_subsystem.stop()

    def isFinished(self) -> bool:
        return False
