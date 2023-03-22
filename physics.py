from pyfrc.physics.core import PhysicsEngine as Engine, PhysicsInterface
from wpilib.simulation import AnalogGyroSim
from wpimath.geometry import Pose2d, Rotation2d
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from wpilib import Field2d
    from robot import Robot


class PhysicsEngine(Engine):
    simGyro = AnalogGyroSim(0)

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
        """Initialize the physics engine with the simulation interface"""

        self.robot = robot
        self.physics_controller = physics_controller

        self.physics_controller.field.setRobotPose(Pose2d(5, 5, Rotation2d(0)))

    def update_sim(self, now: float, tm_diff: float):
        """Called when the simulation parameters for the program need to be updated"""

        if (
            not self.robot.robotContainer.swerveSubsystem.swerveAutoStartPoseUsed
            and self.robot.robotContainer.swerveSubsystem.swerveAutoStartPose
        ):
            self.physics_controller.field.setRobotPose(
                self.robot.robotContainer.swerveSubsystem.swerveAutoStartPose
            )
            self.robot.robotContainer.swerveSubsystem.swerveAutoStartPoseUsed = True

        if self.robot.robotContainer.swerveSubsystem.chassisSpeeds:
            pose = self.physics_controller.drive(
                self.robot.robotContainer.swerveSubsystem.chassisSpeeds, tm_diff
            )

            print(
                "chassisSpeeds",
                self.robot.robotContainer.swerveSubsystem.chassisSpeeds,
                "setting pose",
                pose,
            )

            self.physics_controller.field.setRobotPose(pose)
            # self.robot.robotContainer.swerveSubsystem.odometer.resetPosition()
            self.robot.robotContainer.swerveSubsystem.resetOdometer(pose)

            # print("setting gyro angle", -pose.rotation().degrees())
            self.simGyro.setAngle(pose.rotation().degrees())
