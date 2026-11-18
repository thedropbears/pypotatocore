from typing import override

import commands3
from phoenix6 import swerve
from wpilib import OpMode, autonomous
from wpimath import ChassisVelocities, Rotation2d

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


@autonomous
class DriveForward(OpMode):
    def __init__(self, robot) -> None:
        super().__init__()

        self.drivetrain: CommandSwerveDrivetrain = robot.drivetrain
        self.machine = commands3.StateMachine("Drive Forward Autonomous")

        self._auto_drive = (
            swerve.requests.ApplyFieldVelocity()
            .with_forward_perspective(
                swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        idle_req = swerve.requests.Idle()

        # When auto is selected, reset our field centric heading to match
        # the robot facing away from our alliance station wall (0 deg).
        self.drivetrain.seed_field_centric(Rotation2d())

        # Slowly drive forward (away from us) for 2 seconds.
        drive_forward = self.machine.add_state(
            self.drivetrain.apply_request(
                lambda: self._auto_drive.with_velocity(
                    ChassisVelocities(1.0, 0.0, omega=0.0)
                ),
                name="Auto Drive",
            ).with_timeout(2.0)
        )
        # Idle for the rest of auton
        idle = self.machine.add_state(
            self.drivetrain.apply_request(lambda: idle_req, name="Auto Idle")
        )

        drive_forward.switch_to(idle).when_complete()
        self.machine.set_initial_state(drive_forward)

        self.machine_cmd = self.machine.build()

    @override
    def start(self) -> None:
        commands3.Scheduler.get_default().schedule(self.machine_cmd)
