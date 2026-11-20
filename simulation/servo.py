from wpilib.simulation import DutyCycleEncoderSim
from wpimath import units

from utilities.functions import clamp
from utilities.servo import ServoTurret


class ServoTurretSim:
    """Physical model of a ServoTurret: target angle -> turret angle -> encoder reading."""

    def __init__(
        self,
        turret: ServoTurret,
        max_speed: units.radians_per_second,
        starting_angle: units.radians,
    ) -> None:
        self.turret = turret
        self.encoder_sim = DutyCycleEncoderSim(turret.encoder)
        self.max_speed = max_speed
        self.angle: units.radians = starting_angle
        self._publish()

    def update(self, dt: units.seconds) -> None:
        target = self.turret.last_target_angle

        error = target - self.angle
        max_step = self.max_speed * dt
        self.angle += clamp(error, -max_step, max_step)

        self._publish()

    def _publish(self) -> None:
        # The real encoder reads [0, tau), counting anticlockwise with the turret.
        self.encoder_sim.set(self.turret._angle_to_raw_encoder(self.angle))
