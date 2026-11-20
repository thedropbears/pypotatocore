import math
from dataclasses import dataclass

import wpilib
from rev import ServoChannel, ServoChannelConfig
from wpimath import Rotation2d, TimeInterpolatableRotation2dBuffer, units

from utilities.functions import clamp
from utilities.rev import configure_through_bore_encoder


@dataclass(frozen=True)
class ServoCalibration:
    min_us: int
    center_us: int
    max_us: int
    max_speed: units.radians_per_second


@dataclass
class TurretCalibration:
    """
    Raw encoder readings measured with the turret in known positions.
    Read each one off raw_encoder_reading_() in test mode.
    """

    # Turret pointed forwards by hand (not using the servo). This defines turret angle 0.
    encoder_at_forward: Rotation2d
    # Servo held at center_us, with hold_neutral_()
    encoder_at_center_pulse: Rotation2d
    # Servo held at max_us, with hold_full_range_()
    encoder_at_max_pulse: Rotation2d
    # Further limit to the servo's travel.
    encoder_at_min_limit: Rotation2d
    encoder_at_max_limit: Rotation2d


class ServoTurret:
    HISTORY_LENGTH = 2.0  # s

    def __init__(
        self,
        servo_channel: ServoChannel,
        servo_calibration: ServoCalibration,
        encoder_channel: int,
        turret_calibration: TurretCalibration,
    ) -> None:
        self.servo = servo_channel
        self.servo_calibration = servo_calibration
        self.servo.set_powered(True)
        self.servo.set_enabled(True)

        self.encoder = wpilib.DutyCycleEncoder(encoder_channel, math.tau, 0.0)
        configure_through_bore_encoder(self.encoder)
        self.turret_calibration = turret_calibration

        # Servo geometry as turret angles: a straight line through the neutral point.
        self.neutral_angle = (
            turret_calibration.encoder_at_center_pulse
            - turret_calibration.encoder_at_forward
        ).radians()
        measured_travel = (
            turret_calibration.encoder_at_max_pulse
            - turret_calibration.encoder_at_center_pulse
        ).radians() % math.tau
        self.rad_per_us = measured_travel / (
            servo_calibration.max_us - servo_calibration.center_us
        )

        # Reachable travel is limited by both the pulse range and the mechanism.
        self.min_angle = max(
            (
                turret_calibration.encoder_at_min_limit
                - turret_calibration.encoder_at_forward
            ).radians(),
            self._pulse_to_angle(servo_calibration.min_us),
        )
        self.max_angle = min(
            (
                turret_calibration.encoder_at_max_limit
                - turret_calibration.encoder_at_forward
            ).radians(),
            self._pulse_to_angle(servo_calibration.max_us),
        )

        self.target_angle = self.neutral_angle
        # The angle matching the pulse most recently sent; the sim drives towards this.
        self.last_target_angle = self.neutral_angle
        self._override_pulse: int | None = None
        self._history = TimeInterpolatableRotation2dBuffer(self.HISTORY_LENGTH)

    def set_target(self, turret_angle: units.radians) -> None:
        self.target_angle = self.clamp_angle(turret_angle)

    def clamp_angle(self, turret_angle: units.radians) -> units.radians:
        return clamp(turret_angle, self.min_angle, self.max_angle)

    def channel_config(self) -> ServoChannelConfig:
        """Config for this turret's channel, to apply to the hub's ServoHubConfig."""
        return (
            ServoChannelConfig(self.servo.get_channel_id())
            .pulse_range(
                self.servo_calibration.min_us,
                self.servo_calibration.center_us,
                self.servo_calibration.max_us,
            )
            .disable_behavior(
                ServoChannelConfig.BehaviorWhenDisabled.DO_NOT_SUPPLY_POWER
            )
        )

    def update(self) -> None:
        """Call once per loop: sends the servo pulse and records where the turret is."""
        if self._override_pulse is not None:
            pulse = self._override_pulse
            self._override_pulse = None
        else:
            pulse = self._angle_to_pulse(self.target_angle)

        self.servo.set_pulse_width(pulse)
        self.last_target_angle = self._pulse_to_angle(pulse)

        self._history.add_sample(
            wpilib.Timer.get_monotonic_timestamp(), self.rotation()
        )

    def raw_encoder_reading_(self) -> Rotation2d:
        """The encoder value before turret_calibration is applied. Record this to fill in TurretCalibration."""
        return Rotation2d(self.encoder.get())

    def rotation(self) -> Rotation2d:
        return self.raw_encoder_reading_() - self.turret_calibration.encoder_at_forward

    def rotation_at(self, timestamp: float) -> Rotation2d | None:
        return self._history.sample(timestamp)

    def is_connected(self) -> bool:
        return self.encoder.is_connected()

    def wrap_into_range(self, turret_angle: float) -> float | None:
        wrapped = self.max_angle - ((self.max_angle - turret_angle) % math.tau)
        if wrapped > self.min_angle:
            return wrapped
        return None

    def _angle_to_pulse(self, turret_angle: units.radians) -> int:
        pulse = (
            self.servo_calibration.center_us
            + (turret_angle - self.neutral_angle) / self.rad_per_us
        )
        return round(
            clamp(pulse, self.servo_calibration.min_us, self.servo_calibration.max_us)
        )

    def _pulse_to_angle(self, pulse_us: float) -> units.radians:
        """Turret angle the servo drives to for a given pulse. Inverse of _angle_to_pulse."""
        return (
            self.neutral_angle
            + (pulse_us - self.servo_calibration.center_us) * self.rad_per_us
        )

    def _angle_to_raw_encoder(self, turret_angle: units.radians) -> units.radians:
        return (
            turret_angle + self.turret_calibration.encoder_at_forward.radians()
        ) % math.tau

    def hold_neutral_(self) -> None:
        """Holds the servo at center_us for this loop. Call every loop while calibrating."""
        self._override_pulse = self.servo_calibration.center_us

    def hold_full_range_(self) -> None:
        """Holds the servo at max_us for this loop. Call every loop while calibrating."""
        self._override_pulse = self.servo_calibration.max_us
