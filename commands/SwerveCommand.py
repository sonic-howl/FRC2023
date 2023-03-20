import math
from commands2 import Command, Subsystem
from utils.utils import dz, sign
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds
from wpimath.filter._filter import SlewRateLimiter
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from typing import Callable, Set

from constants import Constants, SwerveConstants
from controllers.pilot import PilotController


class SwerveCommand(Command):
    def __init__(
        self,
        swerve_subsystem: SwerveSubsystem,
        controller: PilotController,
        get_field_oriented: Callable[[], bool],
    ) -> None:
        super().__init__()

        self.swerve_subsystem = swerve_subsystem

        self.controller = controller
        self.get_field_oriented = get_field_oriented

        self.xLimiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.yLimiter = SlewRateLimiter(
            SwerveConstants.kDriveMaxAccelerationMetersPerSecond
        )
        self.zLimiter = SlewRateLimiter(
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

        speed_scale = self.controller.getSpeed()

        x = dz(self.controller.getForward()) * speed_scale
        y = dz(self.controller.getStrafe()) * speed_scale

        z = 0.0

        pov = self.controller.getRotateToAngle()
        if pov != -1:
            # TODO calibrate this PID controller
            z = self.rotate_to_angle_pid.calculate(
                self.swerve_subsystem.getAngle(), pov
            )
        else:
            z = -self.controller.getTurn() * speed_scale
            z = (dz(z) ** 2) * sign(z)
            # z = self.z_limiter.calculate(z)

        # chassis_speeds: ChassisSpeeds | None = None

        magnitude = abs(x) + abs(y) + abs(z)
        if dz(magnitude) > 0:
            # convert values to meters per second and apply rate limiters
            x *= SwerveConstants.kDriveMaxMetersPerSecond
            # x = self.xLimiter.calculate(x)

            y *= SwerveConstants.kDriveMaxMetersPerSecond
            # y = self.yLimiter.calculate(y)

            # z = self.zLimiter.calculate(z)

            if self.get_field_oriented():
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x,
                    y,
                    z,
                    self.swerve_subsystem.getRotation2d(),
                )
            else:
                chassis_speeds = ChassisSpeeds(
                    x,
                    y,
                    z,
                )

            swerve_module_states = (
                SwerveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
            )
            self.swerve_subsystem.setModuleStates(swerve_module_states)
        else:
            self.swerve_subsystem.stop()

    def end(self, interrupted: bool) -> None:
        print("SwerveCommand ended")
        self.swerve_subsystem.stop()

    def isFinished(self) -> bool:
        return False
