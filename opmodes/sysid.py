import commands3.button
from commands3.sysid import SysIdRoutine
from wpilib import OpMode, utility

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


@utility
class SysIdRoutines(OpMode):
    def __init__(self, robot) -> None:
        super().__init__()

        self.drivetrain: CommandSwerveDrivetrain = robot.drivetrain
        self._joystick = commands3.button.CommandXboxController(0)

        # Run SysId routines when holding back (view)/start (menu) and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.view() & self._joystick.y()).while_true(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.FORWARD)
        )
        (self._joystick.view() & self._joystick.x()).while_true(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.REVERSE)
        )
        (self._joystick.menu() & self._joystick.y()).while_true(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.FORWARD)
        )
        (self._joystick.menu() & self._joystick.x()).while_true(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.REVERSE)
        )
