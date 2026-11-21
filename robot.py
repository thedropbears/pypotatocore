#!/usr/bin/env python3


import telemetry
from magicbot import MagicRobot, tunable
from wpilib import Field2d, Timer, XboxController

from components.chassis import ChassisComponent
from swerves.potatobot import tuner_constants
from utilities.scalers import map_exponential


class MyRobot(MagicRobot):
    chassis: ChassisComponent

    # Driving constraints
    max_speed = tunable(0.8)  # m/s
    max_spin_rate = tunable(2.5)  # rad/s

    def create_objects(self) -> None:
        self.gamepad = XboxController(0)

        self.field = Field2d()

        self.chassis_tuner_constants = tuner_constants

        self.sim_loop_timer = Timer()
        self.sim_loop_timer.start()

    def robot_periodic(self) -> None:
        telemetry.log("Field", self.field)

    def disabled_periodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()

    def teleop_periodic(self) -> None:
        drive_speed = self.max_speed
        spin_rate = self.max_spin_rate

        drive_x = -map_exponential(self.gamepad.get_left_y(), 1.5) * drive_speed
        drive_y = -map_exponential(self.gamepad.get_left_x(), 1.5) * drive_speed
        drive_z = -map_exponential(self.gamepad.get_right_x(), 2.0) * spin_rate

        local_driving = self.gamepad.get_right_bumper_button()

        if local_driving:
            self.chassis.drive_robot(drive_x, drive_y, drive_z)
        else:
            self.chassis.drive_field(drive_x, drive_y, drive_z)

    def simulation_init(self) -> None:
        self.sim_loop_timer.reset()

    def simulation_periodic(self) -> None:
        tm_diff = self.sim_loop_timer.get()
        self.sim_loop_timer.reset()

        self.chassis.sim_periodic(tm_diff)
