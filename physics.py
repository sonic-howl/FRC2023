from pyfrc.physics.core import PhysicsEngine as Engine, PhysicsInterface
from wpilib.simulation import AnalogGyroSim
from wpimath.geometry import Pose2d, Rotation2d
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
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

        swerveSubsystem = self.robot.robotContainer.swerveSubsystem

        if swerveSubsystem.swerveAutoStartPose:
            self.physics_controller.field.setRobotPose(
                swerveSubsystem.swerveAutoStartPose
            )
            swerveSubsystem.swerveAutoStartPose = None

        if swerveSubsystem.currentChassisSpeeds:
            # the simulation turning is very slow, speed it up
            swerveSubsystem.currentChassisSpeeds.omega *= 20
            pose = self.physics_controller.drive(
                swerveSubsystem.currentChassisSpeeds, tm_diff
            )

            self.physics_controller.field.setRobotPose(pose)
            swerveSubsystem.resetOdometer(pose)

            self.simGyro.setAngle(pose.rotation().degrees())
