from pyfrc.physics.core import PhysicsEngine as Engine, PhysicsInterface
from pyfrc.physics import drivetrains
from wpilib.simulation import AnalogGyroSim
from wpimath.geometry import Pose2d, Rotation2d

from robot import Robot


class PhysicsEngine(Engine):
    def __init__(self, physics_controller: PhysicsInterface, robot: Robot):
        """Initialize the physics engine with the simulation interface"""

        self.robot = robot
        self.physics_controller = physics_controller

        self.gyro = AnalogGyroSim(0)

        self.physics_controller.field.setRobotPose(Pose2d(5, 5, Rotation2d(0)))

    def update_sim(self, now: float, tm_diff: float):
        """Called when the simulation parameters for the program need to be updated"""

        """
        # not working!
        speeds = drivetrains.four_motor_swerve_drivetrain(
            self.robot.robot_container.swerve_subsystem.back_left.drive_motor.getMotorOutputPercent(),
            -self.robot.robot_container.swerve_subsystem.back_right.drive_motor.getMotorOutputPercent(),
            self.robot.robot_container.swerve_subsystem.front_left.drive_motor.getMotorOutputPercent(),
            -self.robot.robot_container.swerve_subsystem.front_right.drive_motor.getMotorOutputPercent(),
            0,
            180,
            0,
            180,
        )

        pose = self.physics_controller.drive(speeds, tm_diff)

        self.physics_controller.field.setRobotPose(pose)

        self.gyro.setAngle(-pose.rotation().degrees())
        """
