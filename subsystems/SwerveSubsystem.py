from ntcore import NetworkTableInstance
from threading import Thread
from time import sleep
from typing import Tuple

import wpilib
from utils import print_async
from wpimath.kinematics import (
    SwerveModuleState,
    SwerveDrive4Odometry,
)
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpilib import SPI
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
            front_left.get_position(),
            front_right.get_position(),
            back_left.get_position(),
            back_right.get_position(),
        ),
    )

    def __init__(self) -> None:
        super().__init__()

        def reset_gyro():
            """reset gyro after it's calibration of 1s"""
            sleep(1)
            self.gyro.reset()

        Thread(target=reset_gyro).start()

    def get_angle(self):
        # return self.gyro.getAngle() % 360
        # return self.gyro.getFusedHeading()
        return self.gyro.getYaw()

    def get_rotation2d(self):
        return Rotation2d.fromDegrees(self.get_angle())

    def get_pose(self) -> Pose2d:
        return self.odometer.getPose()

    def reset_gyro(self):
        # self.gyro.reset()
        self.gyro.zeroYaw()

    def reset_motor_positions(self):
        self.front_left.reset_encoders()
        self.front_right.reset_encoders()
        self.back_left.reset_encoders()
        self.back_right.reset_encoders()

    def reset_odometer(self, pose: Pose2d = Pose2d()):
        self.odometer.resetPosition(
            self.get_rotation2d(),
            pose,
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.back_left.get_position(),
            self.back_right.get_position(),
        )

    temp_timer = wpilib.Timer()
    last_sensor_pos = 0

    # override
    def periodic(self) -> None:
        # TODO print gyro angle, robot pose on dashboard

        self.odometer.update(
            self.get_rotation2d(),
            self.front_left.get_position(),
            self.front_right.get_position(),
            self.back_left.get_position(),
            self.back_right.get_position(),
        )

        self.temp_timer.start()
        if self.temp_timer.advanceIfElapsed(1):
            if (
                self.front_right.drive_motor.getSelectedSensorPosition()
                != self.last_sensor_pos
            ):

                def do():
                    print(
                        f"""Robot pose: {self.odometer.getPose()}
back_right sensor pos: {self.back_right.drive_motor.getSelectedSensorPosition()}
front right velocity: {self.front_right.drive_motor.getSelectedSensorVelocity()}
front_left : {self.front_left.get_position()}
front_right: {self.front_right.get_position()}
back_left:   {self.back_left.get_position()}
back_right:  {self.back_right.get_position()}
"""
                    )

                Thread(target=do).start()
                self.last_sensor_pos = (
                    self.front_right.drive_motor.getSelectedSensorPosition()
                )

    def stop(self) -> None:
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()

    def setX(self) -> None:
        self.front_left.set_desired_state(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)), True
        )
        self.front_right.set_desired_state(
            SwerveModuleState(0, Rotation2d.fromDegrees(45)), True
        )
        self.back_left.set_desired_state(
            SwerveModuleState(0, Rotation2d.fromDegrees(45)), True
        )
        self.back_right.set_desired_state(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45)), True
        )

    def set_module_states(
        self,
        states: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        fl, fr, bl, br = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, SwerveConstants.kDriveMaxMetersPerSecond
        )
        self.front_left.set_desired_state(fl)
        self.front_right.set_desired_state(fr)
        self.back_right.set_desired_state(bl)
        self.back_left.set_desired_state(br)
