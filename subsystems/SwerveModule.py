from threading import Thread

from wpimath import angleModulus

from ntcore import NetworkTableInstance
import rev
import ctre
import wpilib as wp
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController
import math

from constants import SwerveConstants, Constants


def print_async(*args, **kwargs):
    """print in a new thread"""
    Thread(target=print, args=args, kwargs=kwargs).start()


def scale_speed(speed: float) -> float:
    speed *= Constants.scale_speed
    if speed > Constants.max_speed:
        speed = Constants.max_speed
    elif speed < -Constants.max_speed:
        speed = -Constants.max_speed
    return speed


class SwerveModule:
    def __init__(
        self,
        drive_motor_id: int,
        turn_motor_id: int,
        drive_motor_reversed=False,
        turn_motor_reversed=False,
        abs_encoder_reversed=True,
        abs_encoder_offset_rad=0.0,
        chassis_angular_offset=0.0,
    ) -> None:
        self.chassis_angular_offset = chassis_angular_offset

        # set angle offset
        self.abs_encoder_offset_rad = abs_encoder_offset_rad % (math.pi * 2)
        self.abs_encoder_reversed = abs_encoder_reversed

        self.turn_motor_id = turn_motor_id

        # create motors
        self.drive_motor = ctre.WPI_TalonFX(drive_motor_id)
        self.drive_motor.setInverted(drive_motor_reversed)
        self.turn_motor = rev.CANSparkMax(
            turn_motor_id, rev.CANSparkMaxLowLevel.MotorType.kBrushless
        )
        # ?
        self.turn_motor.restoreFactoryDefaults()
        self.turn_motor.setInverted(turn_motor_reversed)
        self.turn_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.turn_motor.setSmartCurrentLimit(25)

        # create encoders
        self.turn_encoder = self.turn_motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.turn_encoder.setPositionConversionFactor(math.pi * 2)  # radians
        self.turn_encoder.setVelocityConversionFactor(
            (math.pi * 2) / 60
        )  # radians per second
        self.turn_encoder.setInverted(abs_encoder_reversed)
        self.turn_encoder.setZeroOffset(abs_encoder_offset_rad)

        # We are not using the built in PID controller, but this is where the config would go...
        # self. #...
        self.turn_pid = self.turn_motor.getPIDController()
        self.turn_pid.setFeedbackDevice(self.turn_encoder)
        self.turn_pid.setP(SwerveConstants.kPTurning)
        self.turn_pid.setI(SwerveConstants.kITurning)
        self.turn_pid.setD(SwerveConstants.kDTurning)
        self.turn_pid.setPositionPIDWrappingEnabled(True)
        self.turn_pid.setPositionPIDWrappingMinInput(0)
        self.turn_pid.setPositionPIDWrappingMaxInput(math.pi * 2)
        self.turn_pid.setOutputRange(-1, 1)

        self.turn_motor.burnFlash()

        # self.turn_pid = PIDController(
        #     SwerveConstants.kPTurning,
        #     SwerveConstants.kITurning,
        #     SwerveConstants.kDTurning,
        #     Constants.period,
        # )
        # self.turn_pid.enableContinuousInput(-math.pi, math.pi)
        # self.turn_pid.setTolerance(math.radians(SwerveConstants.kTurningToleranceDeg))

        self.reset_encoders()

        self.init_network_tables()

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

    def init_network_tables(self):
        network_table_instance = NetworkTableInstance.getDefault()
        self.swerve_dashboard = network_table_instance.getTable("Swerve Dashboard")
        self.angle_topic_pub = self.swerve_dashboard.getDoubleTopic(
            f"module angle {self.turn_motor_id}"
        ).publish()
        self.drive_speed_topic_pub = self.swerve_dashboard.getDoubleTopic(
            f"module drive speed {self.turn_motor_id}"
        ).publish()
        self.set_point_topic_pub = self.swerve_dashboard.getDoubleTopic(
            f"module set point angle {self.turn_motor_id}"
        ).publish()

        # self.turn_kP_topic = self.swerve_dashboard.getFloatTopic(
        #     f"module kP {self.turn_motor_id}"
        # )
        # self.turn_kP = self.turn_kP_topic.publish()
        # self.turn_kP.setDefault(SwerveConstants.kPTurning)
        # self.turn_kP_sub = self.turn_kP_topic.subscribe(0)

        # self.turn_kI_topic = self.swerve_dashboard.getFloatTopic(
        #     f"module kI {self.turn_motor_id}"
        # )
        # self.turn_kI = self.turn_kI_topic.publish()
        # self.turn_kI.set(SwerveConstants.kITurning)
        # self.turn_kI_sub = self.turn_kI_topic.subscribe(0)

        # self.turn_kD_topic = self.swerve_dashboard.getFloatTopic(
        #     f"module kD {self.turn_motor_id}"
        # )
        # self.turn_kD = self.turn_kD_topic.publish()
        # self.turn_kD.set(SwerveConstants.kDTurning)
        # self.turn_kD_sub = self.turn_kD_topic.subscribe(0)

    def get_absolute_encoder_rad(self) -> float:
        # # voltage to angle
        # # Divide voltage reading by the voltage supplied to it
        # angle = self.abs_encoder.getVoltage() / wp.RobotController.getVoltage5V()
        # # to rad
        # angle *= 2 * math.pi
        # angle -= self.abs_encoder_offset_rad
        return self.turn_encoder.getPosition()

    def reset_encoders(self) -> None:
        # self.drive_encoder.setPosition(0)
        self.drive_motor.setSelectedSensorPosition(0)
        # self.turn_encoder.setPosition(self.get_absolute_encoder_rad())
        # self.drive_encoder.setZeroOffset(self.drive_encoder.getPosition())
        # self.turn_encoder.setZeroOffset(self.get_absolute_encoder_rad())

    def get_rotation(self) -> Rotation2d:
        return Rotation2d(self.get_absolute_encoder_rad())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            # speed=self.drive_encoder.getVelocity(),
            speed=self.drive_motor.getSelectedSensorVelocity(),
            angle=self.get_rotation(),
        )

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            # distance=self.drive_encoder.getPosition(),
            distance=self.drive_motor.getSelectedSensorPosition(),
            angle=self.get_rotation(),
        )

    def set_desired_state(self, state: SwerveModuleState) -> None:
        if abs(state.speed) < 0.01:
            self.stop()
            return

        state.angle = Rotation2d(state.angle.radians() + self.chassis_angular_offset)

        # optimize AFTER adding the chassis angular offset
        state = SwerveModuleState.optimize(state, self.get_rotation())

        # TODO make a drive velocity PID instead of scaling it this way
        drive_speed = state.speed / SwerveConstants.kDriveMaxMetersPerSecond
        drive_speed = scale_speed(drive_speed)
        self.drive_motor.set(drive_speed)

        set_point = float(state.angle.radians())
        current_angle = self.get_absolute_encoder_rad()

        # turn_speed = self.turn_pid.calculate(current_angle, set_point)
        # turn_speed = scale_speed(turn_speed)

        self.angle_topic_pub.set(math.degrees(current_angle))
        self.drive_speed_topic_pub.set(drive_speed)

        # self.turn_motor.set(turn_speed)
        self.turn_pid.setReference(set_point, rev.CANSparkMax.ControlType.kPosition)

    def stop(self) -> None:
        self.drive_motor.stopMotor()
        self.turn_motor.stopMotor()
