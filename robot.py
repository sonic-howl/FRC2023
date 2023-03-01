import math
import wpilib as wp
from subsystems.SwerveModule import SwerveModule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import rev
from commands2 import CommandScheduler

# from networktables import NetworkTables
from ntcore import NetworkTableInstance

import rev


from commands.SwerveCommand import SwerveCommand
from RobotContainer import RobotContainer

from constants import Constants


class Robot(wp.TimedRobot):
    def __init__(self, period=Constants.period) -> None:
        super().__init__(period)

    def robotInit(self) -> None:
        # self.smartDashboard = NetworkTables.getTable("SmartDashboard")
        network_table_instance = NetworkTableInstance.getDefault()
        self.smartDashboard = network_table_instance.getTable("SmartDashboard")
        self.gyroTopic = self.smartDashboard.getFloatTopic("Gyro Angle").publish()
        self.turner_topic = self.smartDashboard.getFloatTopic("Turn Encoder").publish()
        # # create ps4 controller
        # self.controller = wp.PS4Controller(0)

        # # self.module_1 = SwerveModule()
        # self.turner = rev.CANSparkMax(3, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        # self.turner_encoder = self.turner.getAbsoluteEncoder(
        #     rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        # )

        self.robot_container = RobotContainer()

    def robotPeriodic(self) -> None:
        self.gyroTopic.set(self.robot_container.get_angle())

        self.turner_topic.set(
            # self.robot_container.swerve_subsystem.front_left.turn_encoder.getPosition()
            # % math.pi
            # * 2
            self.robot_container.swerve_subsystem.front_left.get_position().angle.degrees()
        )

        try:
            CommandScheduler.getInstance().run()
        except:
            print("CommandScheduler error")

    def autonomousInit(self) -> None:
        auto_command = self.robot_container.getAutonomousCommand()
        if auto_command is not None:
            auto_command.schedule()
            print("Auto command scheduled")

    def teleopPeriodic(self) -> None:
        # TODO maybe stop the auto command here if executing the swerve drive command doesn't stop it.
        # There may not need to be any teleop code here if everything uses commands.
        # The commands are run using the command scheduler which is run in robotPeriodic.
        # The CommandScheduler automatically runs the appropriate code depending on the state of the competition (disabled, autonomous, teleoperated).
        pass


if __name__ == "__main__":
    wp.run(Robot)
