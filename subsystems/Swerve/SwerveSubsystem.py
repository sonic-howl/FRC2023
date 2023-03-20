from ntcore import NetworkTableInstance
from threading import Thread
from time import sleep
from typing import Tuple

import wpilib
from utils.utils import printAsync
from wpimath.kinematics import (
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

    gyro = AHRS(SPI.Port.kMXP, int(1 / Constants.period))

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

        def reset_gyro():
            """reset gyro after it's calibration of 1s"""
            sleep(1)
            self.gyro.reset()

        Thread(target=reset_gyro).start()

        # self.field = Field2d()
        # SmartDashboard.putData("Field", self.field)

    def getAngle(self):
        # return self.gyro.getAngle() % 360
        # return self.gyro.getFusedHeading()
        return -self.gyro.getYaw()

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngle())

    def getPose(self) -> Pose2d:
        return self.odometer.getPose()

    def reset_gyro(self):
        # self.gyro.reset()
        self.gyro.zeroYaw()

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

    temp_timer = wpilib.Timer()
    last_sensor_pos = 0

    # override
    def periodic(self) -> None:
        # TODO print gyro angle, robot pose on dashboard

        # self.field.setRobotPose(self.getPose())

        self.odometer.update(
            self.getRotation2d(),
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition(),
        )

    #         self.temp_timer.start()
    #         if self.temp_timer.advanceIfElapsed(1):
    #             if (
    #                 self.front_right.drive_motor.getSelectedSensorPosition()
    #                 != self.last_sensor_pos
    #             ):

    #                 def do():
    #                     print(
    #                         f"""Robot pose: {self.odometer.getPose()}
    # back_right sensor pos: {self.back_right.drive_motor.getSelectedSensorPosition()}
    # front right velocity: {self.front_right.drive_motor.getSelectedSensorVelocity()}
    # front_left : {self.front_left.get_position()}
    # front_right: {self.front_right.get_position()}
    # back_left:   {self.back_left.get_position()}
    # back_right:  {self.back_right.get_position()}
    # """
    #                     )

    #                 Thread(target=do).start()
    #                 self.last_sensor_pos = (
    #                     self.front_right.drive_motor.getSelectedSensorPosition()
    #                 )

    def stop(self) -> None:
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()

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
