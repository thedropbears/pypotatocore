import wpilib
from phoenix6 import CANBus, configs, signals, swerve
from wpimath.units import inches_to_meters

from swerves.tuner_constants import TunerConstants

tuner_constants = TunerConstants(
    # Both sets of gains need to be tuned to your individual robot
    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains=(
        configs.Slot0Configs()
        .with_k_p(100)
        .with_k_i(0)
        .with_k_d(0.5)
        .with_k_s(0.1)
        .with_k_v(2.66)
        .with_k_a(0)
        .with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
    ),
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains=(
        configs.Slot0Configs()
        .with_k_p(0.2)
        .with_k_i(0)
        .with_k_d(0)
        .with_k_s(0)
        .with_k_v(0.124)
    ),
    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output=swerve.ClosedLoopOutputType.VOLTAGE,
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output=swerve.ClosedLoopOutputType.VOLTAGE,
    # The type of motor used for the drive motor
    _drive_motor_type=swerve.DriveMotorArrangement.TALON_FX_INTEGRATED,
    # The type of motor used for the steer motor
    _steer_motor_type=swerve.SteerMotorArrangement.TALON_FX_INTEGRATED,
    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    _steer_feedback_type=swerve.SteerFeedbackType.FUSED_CANCODER,
    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot
    _slip_current=120.0,
    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    # Default supply current limit is 70 A, but it can be lowered to avoid brownouts.
    # Supply current limits can be larger than the breaker current rating.
    _drive_initial_configs=configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_supply_current_limit(70.0)
        .with_supply_current_limit_enable(True)
    ),
    # Swerve azimuth does not require much torque output, so we can set a relatively low
    # stator current limit to help avoid brownouts without impacting performance.
    _steer_initial_configs=configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(60.0)
        .with_stator_current_limit_enable(True)
    ),
    _encoder_initial_configs=configs.CANcoderConfiguration(),
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs=None,
    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus=CANBus(wpilib.CANPort.CAN_S4),
    # Measured robot speed (m/s) at 12 V applied output;
    # This is NOT the desired max robot speed - see _max_speed in RobotContainer instead;
    # This needs to be tuned to your individual robot
    speed_at_12_volts=3.79,
    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    _couple_ratio=3.5714285714285716,
    _drive_gear_ratio=8.142857142857142,
    _steer_gear_ratio=21.428571428571427,
    _wheel_radius=inches_to_meters(2),
    _invert_left_side=False,
    _invert_right_side=True,
    _pigeon_id=0,
    # These are only used for simulation
    _steer_inertia=0.01,
    _drive_inertia=0.035,
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage=0.2,
    _drive_friction_voltage=0.2,
    # Front Left
    _front_left_drive_motor_id=1,
    _front_left_steer_motor_id=5,
    _front_left_encoder_id=1,
    _front_left_encoder_offset=0.35791015625,
    _front_left_steer_motor_inverted=True,
    _front_left_encoder_inverted=False,
    _front_left_x_pos=inches_to_meters(9.25),
    _front_left_y_pos=inches_to_meters(9.25),
    # Front Right
    _front_right_drive_motor_id=4,
    _front_right_steer_motor_id=8,
    _front_right_encoder_id=2,
    _front_right_encoder_offset=-0.016357421875,
    _front_right_steer_motor_inverted=True,
    _front_right_encoder_inverted=False,
    _front_right_x_pos=inches_to_meters(9.25),
    _front_right_y_pos=inches_to_meters(-9.25),
    # Back Left
    _back_left_drive_motor_id=2,
    _back_left_steer_motor_id=6,
    _back_left_encoder_id=4,
    _back_left_encoder_offset=-0.001708984375,
    _back_left_steer_motor_inverted=True,
    _back_left_encoder_inverted=False,
    _back_left_x_pos=inches_to_meters(-9.25),
    _back_left_y_pos=inches_to_meters(9.25),
    # Back Right
    _back_right_drive_motor_id=3,
    _back_right_steer_motor_id=7,
    _back_right_encoder_id=3,
    _back_right_encoder_offset=0.183837890625,
    _back_right_steer_motor_inverted=True,
    _back_right_encoder_inverted=False,
    _back_right_x_pos=inches_to_meters(-9.25),
    _back_right_y_pos=inches_to_meters(-9.25),
)
