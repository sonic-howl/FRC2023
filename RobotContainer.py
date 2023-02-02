import wpilib as wp
from commands2 import RunCommand, InstantCommand
from commands2.button import JoystickButton, CommandPS4Controller

from constants import Constants
from subsystems.SwerveSubsystem import SwerveSubsystem
from commands.SwerveCommand import SwerveCommand


class RobotContainer:
    swerve_subsystem = SwerveSubsystem()

    controller = CommandPS4Controller(Constants.pilot_controller_id)

    field_oriented = False

    right_stick_sets_angle = True

    def __init__(self) -> None:
        self.configure_button_bindings()

        # self.set_right_stick_turn_command()
        self.swerve_subsystem.setDefaultCommand(
            SwerveCommand(
                self.swerve_subsystem,
                self.controller,
                self.get_right_stick_sets_angle,
                self.get_field_oriented,
            )
        )

    def get_right_stick_sets_angle(self) -> bool:
        return self.right_stick_sets_angle

    def teleopPeriodic(self) -> None:
        if self.controller.getSquareButtonPressed():
            self.toggle_field_oriented()

    def get_field_oriented(self) -> bool:
        return self.field_oriented

    def toggle_field_oriented(self) -> None:
        self.field_oriented = not self.field_oriented
        print("Field oriented: ", self.field_oriented)

    def configure_button_bindings(self) -> None:
        self.controller.square().onTrue(InstantCommand(self.toggle_field_oriented))
        # JoystickButton(self.controller, 1).onTrue(
        #     InstantCommand(self.toggle_field_oriented)
        # )
