from phoenix6 import swerve

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from swerve_telemetry import SwerveTelemetry
from swerves.potatobot import tuner_constants


class RobotContainer:
    # TODO: move everything left here into robot.py

    def __init__(self) -> None:
        self._logger = SwerveTelemetry(tuner_constants.speed_at_12_volts)

        self._idle = swerve.requests.Idle()

        self.drivetrain = CommandSwerveDrivetrain(
            tuner_constants.drivetrain_constants,
            [
                tuner_constants.front_left,
                tuner_constants.front_right,
                tuner_constants.back_left,
                tuner_constants.back_right,
            ],
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def periodic(self) -> None:
        self.drivetrain.periodic()

    def disabled_periodic(self) -> None:
        # commands3 cancels mechanism commands while disabled. Continue
        # applying the safe idle request without scheduling a command.
        self.drivetrain.set_control(self._idle)
