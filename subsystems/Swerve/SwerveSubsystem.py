from threading import Thread
from time import sleep
from typing import Tuple

import wpilib
from ntcore import NetworkTableInstance
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Odometry,
)
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpilib import SPI, Field2d, SmartDashboard
from navx import AHRS
from commands2 import SubsystemBase

from .SwerveModule import SwerveModule
from constants import SwerveConstants, Constants


class SwerveSubsystem(SubsystemBase):
    simChassisSpeeds: ChassisSpeeds | None = None
    """Meant for simulation only"""
    swerveAutoStartPose: Pose2d | None = None
    """Meant for simulation only"""

    front_left = SwerveModule(
        SwerveConstants.fl_drive_id,
        SwerveConstants.fl_turn_id,
        abs_encoder_offset_rad=SwerveConstants.fl_abs_encoder_offset_rad,
        chassis_angular_offset=SwerveConstants.fl_chassis_angular_offset,
    )
    front_right = SwerveModule(
        SwerveConstants.fr_drive_id,
        SwerveConstants.fr_turn_id,
        abs_encoder_offset_rad=SwerveConstants.fr_abs_encoder_offset_rad,
        chassis_angular_offset=SwerveConstants.fr_chassis_angular_offset,
    )
    back_left = SwerveModule(
        SwerveConstants.bl_drive_id,
        SwerveConstants.bl_turn_id,
        abs_encoder_offset_rad=SwerveConstants.bl_abs_encoder_offset_rad,
        chassis_angular_offset=SwerveConstants.bl_chassis_angular_offset,
    )
    back_right = SwerveModule(
        SwerveConstants.br_drive_id,
        SwerveConstants.br_turn_id,
        abs_encoder_offset_rad=SwerveConstants.br_abs_encoder_offset_rad,
        chassis_angular_offset=SwerveConstants.br_chassis_angular_offset,
    )

    gyro = (
        AHRS(
            wpilib.SerialPort.Port.kUSB,
            AHRS.SerialDataType.kProcessedData,
            int(1 / Constants.period),
        )
        if Constants.navxPort == Constants.NavXPort.kUSB
        else AHRS(
            wpilib.SPI.Port.kMXP,
            int(1 / Constants.period),
        )
    )

    odometer = SwerveDrive4Odometry(
        SwerveConstants.kDriveKinematics,
        Rotation2d(),
        (
            front_left.getPosition(),
            front_right.getPosition(),
            back_left.getPosition(),
            back_right.getPosition(),
        ),
    )

    def __init__(self) -> None:
        super().__init__()

        def resetGyro():
            """reset gyro after it's calibration of 1s"""
            sleep(1)
            self.resetGyro()

        Thread(target=resetGyro).start()

        if not Constants.isSimulation:
            self.field = Field2d()
            SmartDashboard.putData("Field", self.field)

    def getAngle(self) -> float:
        # return self.gyro.getAngle() % 360
        # return self.gyro.getFusedHeading()
        if Constants.isSimulation:
            from physics import PhysicsEngine

            return PhysicsEngine.simGyro.getAngle()

        return -self.gyro.getYaw()

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngle())

    def getPose(self) -> Pose2d:
        return self.odometer.getPose()

    def resetGyro(self):
        # self.gyro.zeroYaw()
        self.gyro.reset()

    def reset_motor_positions(self):
        self.front_left.resetEncoders()
        self.front_right.resetEncoders()
        self.back_left.resetEncoders()
        self.back_right.resetEncoders()

    def resetOdometer(self, pose: Pose2d = Pose2d()):
        self.odometer.resetPosition(
            self.getRotation2d(),
            pose,
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition(),
        )

    def periodic(self) -> None:
        # TODO print gyro angle, robot pose on dashboard

        if not Constants.isSimulation:
            self.field.setRobotPose(self.getPose())

        self.odometer.update(
            self.getRotation2d(),
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition(),
        )

    def stop(self) -> None:
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()

        if Constants.isSimulation:
            self.simChassisSpeeds = None

    def setX(self) -> None:
        self.front_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)), True
        )
        self.front_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45)), True
        )
        self.back_left.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45)), True
        )
        self.back_right.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)), True
        )

    @staticmethod
    def toSwerveModuleStatesForecast(chassisSpeeds: ChassisSpeeds):
        """
        Forecast the swerve module states based on the chassis speeds and the period rather than sending the current chassis speeds.
        This helps to keep the robot moving in a straight line while spinning.

        Thanks to 254: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
        """

        robotPoseVel = Pose2d(
            chassisSpeeds.vx * Constants.period,
            chassisSpeeds.vy * Constants.period,
            Rotation2d(chassisSpeeds.omega * Constants.period),
        )
        twistVel = Pose2d().log(robotPoseVel)
        updatedChassisSpeeds = ChassisSpeeds(
            twistVel.dx / Constants.period,
            twistVel.dy / Constants.period,
            twistVel.dtheta / Constants.period,
        )

        return SwerveConstants.kDriveKinematics.toSwerveModuleStates(
            updatedChassisSpeeds
        )

    def setModuleStates(
        self,
        states: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
        isClosedLoop=False,
    ) -> None:
        fl, fr, bl, br = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, SwerveConstants.kDriveMaxMetersPerSecond
        )
        self.front_left.setDesiredState(fl, isClosedLoop=isClosedLoop)
        self.front_right.setDesiredState(fr, isClosedLoop=isClosedLoop)
        self.back_right.setDesiredState(bl, isClosedLoop=isClosedLoop)
        self.back_left.setDesiredState(br, isClosedLoop=isClosedLoop)
