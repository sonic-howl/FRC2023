import rev
import wpilib as wp
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
import math

from constants import SwerveConstants, Constants


class SwerveModule:
    drive_motor: rev.CANSparkMax
    turn_motor: rev.CANSparkMax

    drive_encoder: rev.AbsoluteEncoder
    turn_encoder: rev.AbsoluteEncoder

    turn_pid: PIDController

    abs_encoder: wp.AnalogInput
    abs_encoder_reversed: bool
    abs_encoder_offset_rad: float

    def __init__(
        self,
        drive_motor_id: int,
        turn_motor_id: int,
        abs_encoder_id: int,
        drive_motor_reversed=False,
        turn_motor_reversed=False,
        abs_encoder_reversed=False,
        abs_encoder_offset_rad=0.0,
        period=0.02,
    ) -> None:
        # set angle offset
        self.abs_encoder_offset_rad = abs_encoder_offset_rad
        self.abs_encoder_reversed = abs_encoder_reversed

        # create motors
        self.drive_motor = rev.CANSparkMax(
            drive_motor_id, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.drive_motor.setInverted(drive_motor_reversed)
        self.turn_motor = rev.CANSparkMax(
            turn_motor_id, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        self.turn_motor.setInverted(turn_motor_reversed)

        # create encoders
        self.abs_encoder = wp.AnalogInput(abs_encoder_id)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.turn_encoder = self.turn_motor.getEncoder()

        self.drive_encoder.setPositionConversionFactor(
            SwerveConstants.kDriveEncoderRotToMeters
        )
        self.drive_encoder.setVelocityConversionFactor(
            SwerveConstants.kDriveEncoderRotToVelocityMps
        )
        self.turn_encoder.setPositionConversionFactor(
            SwerveConstants.kTurnEncoderRotToMeters
        )
        self.turn_encoder.setPositionConversionFactor(
            SwerveConstants.kTurnEncoderRotToVelocityMps
        )

        self.turn_pid = PIDController(SwerveConstants.kPTurning, 0, 0, period)
        self.turn_pid.enableContinuousInput(-math.pi, math.pi)

        self.reset_encoders()

        # def __init__(self, chassis_angular_offset=0) -> None:
        #     # set angle offset
        #     self.chassis_angular_offset = chassis_angular_offset

        #     # create motors
        #     # TODO add id as init args
        #     self.turner = rev.CANSparkMax(1, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        #     self.driver = rev.CANSparkMax(0, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        #     # config encoders and PID controllers
        #     self.driver_encoder = self.driver.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        #     self.driver_pid = self.driver.getPIDController()
        #     self.driver_pid.setFeedbackDevice(self.driver_encoder)
        #     self.turn_encoder = self.turner.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        #     self.turner_pid = self.turner.getPIDController()
        #     self.turner_pid.setFeedbackDevice(self.turn_encoder)

        # def set_state(self, desired_state: SwerveModuleState):
        #     '''// Apply chassis angular offset to the desired state.
        #     SwerveModuleState correctedDesiredState = new SwerveModuleState();
        #     correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        #     correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        #     // Optimize the reference state to avoid spinning further than 90 degrees.
        #     SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        #         new Rotation2d(m_turningEncoder.getPosition()));

        #     // Command driving and turning SPARKS MAX towards their respective setpoints.
        #     m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        #     m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        #     m_desiredState = desiredState;
        #     '''

        #     corrected_state = SwerveModuleState()
        #     corrected_state.speed = desired_state.speed
        #     corrected_state.speed = desired_state.angle + self.chassis_angular_offset

        #     optimized_state = SwerveModuleState.optimize(corrected_state, Rotation2d(self.turn_encoder.getPosition()))

        #     self.driver_pid.setReference(optimized_state.speed, rev.CANSparkMax.ControlType.kVelocity)
        #     self.turner_pid.setReference(optimized_state.angle.getRadians(), rev.CANSparkMax.ControlType.kPosition)

    def get_absolute_encoder_rad(self) -> float:
        # voltage to angle
        # Divide voltage reading by the voltage supplied to it
        angle = self.abs_encoder.getVoltage() / wp.RobotController.getVoltage5V()
        # to rad
        angle *= 2 * math.pi
        angle -= self.abs_encoder_offset_rad
        if self.abs_encoder_reversed:
            angle *= -1
        return angle

    def reset_encoders(self) -> None:
        self.drive_encoder.setPosition(0)
        self.turn_encoder.setPosition(self.get_absolute_encoder_rad())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            speed=self.drive_encoder.getVelocity(),
            angle=Rotation2d(self.get_absolute_encoder_rad()),
        )

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            distance=self.drive_encoder.getPosition(),
            angle=Rotation2d(self.get_absolute_encoder_rad()),
        )

    def set_desired_state(self, state: SwerveModuleState) -> None:
        if abs(state.speed) < 0.001:
            self.stop()
            return

        state = SwerveModuleState.optimize(state, self.get_state().angle)
        self.drive_motor.set(state.speed / SwerveConstants.kDriveMaxMetersPerSecond)
        self.turn_motor.set(
            self.turn_pid.calculate(
                self.turn_encoder.getPosition(), state.angle.radians()
            )
        )

    def stop(self) -> None:
        self.drive_motor.set(0)
        self.turn_motor.set(0)
