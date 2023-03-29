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
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib import SPI, Field2d, SmartDashboard
from navx import AHRS
from commands2 import SubsystemBase

from .SwerveModule import SwerveModule
from constants import SwerveConstants, Constants


class SwerveSubsystem(SubsystemBase):
    currentChassisSpeeds: ChassisSpeeds | None = None
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

    def __init__(self) -> None:
        super().__init__()

        self.gyro = (
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

        ntInstance = NetworkTableInstance.getDefault()
        self.limelightTable = ntInstance.getTable("limelight")
        self.tv = self.limelightTable.getIntegerTopic('tv').subscribe(0)
        self.ta = self.limelightTable.getDoubleTopic('ta').subscribe(0.0)

        self.odometer = SwerveDrive4PoseEstimator(
            SwerveConstants.kDriveKinematics,
            Rotation2d(),
                        (
                self.front_left.getPosition(),
                self.front_right.getPosition(),
                self.back_left.getPosition(),
                self.back_right.getPosition(),
            ),
            Pose2d()
        )
        


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
        return self.odometer.getEstimatedPosition()

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
            (
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition(),
            ),
            pose,
        )

    def periodic(self) -> None:
        # TODO print gyro angle, robot pose on dashboard

        if not Constants.isSimulation:
            self.field.setRobotPose(self.getPose())

        # Correct for pose when an april tag is in view of the limelight at a certain distance
        if self.tv.get() and self.ta.get() > SwerveConstants.minimumAprilTagArea:
            robotPose = self.limelightTable.getDoubleArrayTopic('botpose')
            self.odometer.addVisionMeasurement(
                Pose2d(robotPose[0],robotPose[1],robotPose[2]),
                self.limelightTable.getFloatTopic('ts') / 1000 # Outputs milliseconds, requiers seconds
            )
            # Debug
            SmartDashboard.putData("Estimate robot pose", self.odometer.getEstimatedPosition())

        self.odometer.update(
            self.getRotation2d(),
            (
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition(),
            ),
        )

    def stop(self) -> None:
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()

        if Constants.isSimulation:
            self.currentChassisSpeeds = None

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
