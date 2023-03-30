import math
from typing import Callable

from commands2 import CommandBase
from constants.VisionConstants import VisionConstants
from subsystems.Swerve.LLTable import LLTable
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds

from constants.RobotConstants import RobotConstants
from constants.SwerveConstants import SwerveConstants
from controllers.pilot import PilotController
from subsystems.Swerve.SwerveSubsystem import SwerveSubsystem
from utils.utils import calcAxisSpeedWithCurvatureAndDeadzone, dz


class SwerveCommand(CommandBase):
    def __init__(
        self,
        swerveSubsystem: SwerveSubsystem,
        controller: PilotController,
        getFieldOriented: Callable[[], bool],
    ) -> None:
        super().__init__()

        self.swerveSubsystem = swerveSubsystem

        self.controller = controller
        self.get_field_oriented = getFieldOriented

        # self.xLimiter = SlewRateLimiter(
        #     SwerveConstants.kDriveXLimit, -SwerveConstants.kDriveXLimit
        # )
        # self.yLimiter = SlewRateLimiter(SwerveConstants.kDriveYLimit)
        self.zLimiter = SlewRateLimiter(SwerveConstants.kDriveZLimit)

        self.ll = LLTable.getInstance()
        self.visionTriggered = False

        self.addRequirements(swerveSubsystem)

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
            z = self.swerveSubsystem.thetaPID.calculate(
                math.radians(self.swerveSubsystem.getAngle()),
                math.radians(360 - pov),
            )
            # print("pov debug: ", pov, self.swerveSubsystem.getAngle(), z)
        else:
            z = self.controller.getTurn() * speed_scale
            # z = self.zLimiter.calculate(z)
            z = calcAxisSpeedWithCurvatureAndDeadzone(z) * RobotConstants.rotationScale

        # VISION TRACKING
        # !
        # TODO test this
        # this tracks the target and strafes horizontally to the target.
        # it does not rotate the robot to face the target. That's done by the POV above by the driver.
        visionYMps = 0
        if self.controller.getTrackTargetBtn():
            if self.ll.getTv():
                if not self.visionTriggered:
                    VisionConstants.xyVisionPID.reset()
                    self.visionTriggered = True

                tx = self.ll.getTxScaled()
                visionYMps += VisionConstants.xyVisionPID.calculate(tx, 0)
        else:
            self.visionTriggered = False

        magnitude = abs(x) + abs(y) + abs(z) + abs(visionYMps)
        if dz(magnitude, 0.05) > 0:
            # convert values to meters per second and apply rate limiters
            x *= SwerveConstants.kDriveMaxMetersPerSecond
            # x = self.xLimiter.calculate(x)

            y *= SwerveConstants.kDriveMaxMetersPerSecond
            # y = self.yLimiter.calculate(y)

            # print("y:", y, "visionYMps:", visionYMps)
            # y is up and down on the field. X is left and right from the camera's perspective.
            y += visionYMps

            z = self.zLimiter.calculate(z)

            if self.get_field_oriented():
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x,
                    y,
                    z,
                    self.swerveSubsystem.getRotation2d(),
                )
            else:
                chassisSpeeds = ChassisSpeeds(
                    x,
                    y,
                    z,
                )

            if RobotConstants.isSimulation:
                self.swerveSubsystem.simChassisSpeeds = chassisSpeeds

            swerveModuleStates = SwerveSubsystem.toSwerveModuleStatesForecast(
                chassisSpeeds
            )
            self.swerveSubsystem.setModuleStates(swerveModuleStates)
        else:
            if RobotConstants.isSimulation:
                self.swerveSubsystem.simChassisSpeeds = None

            self.swerveSubsystem.stop()

    def end(self, interrupted: bool) -> None:
        print("SwerveCommand ended")
        self.swerveSubsystem.stop()

    def isFinished(self) -> bool:
        return False
