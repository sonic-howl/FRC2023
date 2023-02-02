import wpilib as wp
from subsystems.SwerveModule import SwerveModule
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
import rev
from commands2 import CommandScheduler

from commands.SwerveCommand import SwerveCommand
from RobotContainer import RobotContainer

from constants import Constants


class Robot(wp.TimedRobot):
    def __init__(self, period=Constants.period) -> None:
        super().__init__(period)

    def robotInit(self) -> None:
        # # create ps4 controller
        # self.controller = wp.PS4Controller(0)

        # # self.module_1 = SwerveModule()
        # self.turner = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        # self.turner_encoder = self.turner.getAbsoluteEncoder(
        #     rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        # )

        self.robot_container = RobotContainer()

    def robotPeriodic(self) -> None:
        # print(self.encoder.getPosition())

        CommandScheduler.getInstance().run()

    def teleopPeriodic(self) -> None:
        pass

        # self.turner.set(self.controller.getLeftY())
        # print(self.turner_encoder.getPosition())

        # self.module_1.set_state(SwerveModuleState(self.controller.getLeftY(), Rotation2d(self.controller.getRightY())))
        # for testing
        # self.motor.set(self.controller.getLeftY())


if __name__ == "__main__":
    wp.run(Robot)
