import commands3.button
from phoenix6 import swerve
from wpilib import OpMode, teleop
from wpimath import Rotation2d, units

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from swerves.potatobot import tuner_constants


@teleop
class DriverControl(OpMode):
    def __init__(self, robot) -> None:
        super().__init__()

        self.drivetrain: CommandSwerveDrivetrain = robot.drivetrain
        self._joystick = commands3.button.CommandXboxController(0)

        self._max_speed = 0.25 * tuner_constants.speed_at_12_volts
        self._max_angular_rate = units.rotations_to_radians(0.75)

        self._drive = (
            swerve.requests.FieldCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self.drivetrain.set_default_command(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.get_left_y() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.get_left_x() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.get_right_x() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                ),
                name="Teleop Drive",
                priority=commands3.LOWEST_PRIORITY,
            )
        )

        self._joystick.a().while_true(
            self.drivetrain.apply_request(lambda: self._brake, name="Brake")
        )
        self._joystick.b().while_true(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(
                        -self._joystick.get_left_y(), -self._joystick.get_left_x()
                    )
                ),
                name="Point Wheels",
            )
        )

        # reset the field-centric heading on left bumper press
        self._joystick.left_bumper().on_true(
            self.drivetrain.run(self._seed_field_centric).named(
                "Reset Field Centric Heading"
            )
        )

    async def _seed_field_centric(self) -> None:
        self.drivetrain.seed_field_centric()
