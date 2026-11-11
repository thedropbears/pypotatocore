#!/usr/bin/env python3


import telemetry
from magicbot import MagicRobot, tunable
from wpilib import Field2d, Timer, XboxController

from components.chassis import ChassisComponent
from swerves.potatobot import tuner_constants
from utilities.scalers import rescale_js


class MyRobot(MagicRobot):
    chassis: ChassisComponent

    # Driving constraints
    max_speed = tunable(0.8)  # m/s
    max_spin_rate = tunable(2.5)  # rad/s

    def __init__(self) -> None:

        super().__init__()

    def create_objects(self) -> None:
        self.gamepad = XboxController(0)

        self.field = Field2d()

        self.chassis_tuner_constants = tuner_constants

    def robot_periodic(self) -> None:
        telemetry.log("Field", self.field)

    def disabled_init(self) -> None:
        pass

    def disabled_periodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

    def disabled_exit(self) -> None:
        pass

    def teleop_init(self) -> None:
        pass

    def teleop_periodic(self) -> None:
        drive_speed = self.max_speed
        spin_rate = self.max_spin_rate

        drive_x = -rescale_js(self.gamepad.get_left_y(), 0.05, 1.5) * drive_speed
        drive_y = -rescale_js(self.gamepad.get_left_x(), 0.05, 1.5) * drive_speed
        drive_z = (
            -rescale_js(self.gamepad.get_right_x(), 0.1, exponential=2.0) * spin_rate
        )

        local_driving = self.gamepad.get_right_bumper_button()

        if local_driving:
            self.chassis.drive_robot(drive_x, drive_y, drive_z)
        else:
            self.chassis.drive_field(drive_x, drive_y, drive_z)

    def teleop_exit(self) -> None:
        pass

    def simulation_init(self) -> None:
        # Called once, after create_objects and component injection
        self._last_sim_time = Timer.get_timestamp()

    def simulation_periodic(self) -> None:
        now = Timer.get_timestamp()
        tm_diff = now - self._last_sim_time
        self._last_sim_time = now

        self.chassis.sim_periodic(tm_diff)
