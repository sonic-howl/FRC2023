from threading import Thread
from time import sleep
from typing import Tuple
from wpimath.kinematics import (
    SwerveModuleState,
    SwerveDrive4Odometry,
    SwerveModulePosition,
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
        SwerveConstants.fl_abs_encoder_id,
        period=Constants.period,
    )
    front_right = SwerveModule(
        SwerveConstants.fr_drive_id,
        SwerveConstants.fr_turn_id,
        SwerveConstants.fr_abs_encoder_id,
        period=Constants.period,
    )
    back_right = SwerveModule(
        SwerveConstants.br_drive_id,
        SwerveConstants.br_turn_id,
        SwerveConstants.br_abs_encoder_id,
        period=Constants.period,
    )
    back_left = SwerveModule(
        SwerveConstants.bl_drive_id,
        SwerveConstants.bl_turn_id,
        SwerveConstants.bl_abs_encoder_id,
        period=Constants.period,
    )

    gyro = AHRS(SPI.Port.kMXP, int(1000 / (Constants.period * 1000)))

    odometer = SwerveDrive4Odometry(
        SwerveConstants.kDriveKinematics,
        Rotation2d(0),
        (
            front_left.get_position(),
            front_right.get_position(),
            back_right.get_position(),
            back_left.get_position(),
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
        return self.gyro.getYaw()

    def get_rotation2d(self):
        return Rotation2d.fromDegrees(self.get_angle())

    def get_pose(self):
        return self.odometer.getPose()

    # def reset_odometer(self, pose: Pose2d):
    #     self.odometer.resetPosition(self.get_rotation2d(), pose, self.front_left.) # TODO get the positions of the modules

    # override
    def periodic(self) -> None:
        # TODO print gyro angle on dashboard
        pass

    def stop(self) -> None:
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()

    def set_module_states(
        self,
        states: Tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, SwerveConstants.kWheelMaxSpeedMetersPerSecond
        )
        self.front_left.set_desired_state(states[0])
        self.front_right.set_desired_state(states[1])
        self.back_left.set_desired_state(states[2])
        self.back_right.set_desired_state(states[3])
