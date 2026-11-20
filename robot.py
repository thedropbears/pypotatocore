#!/usr/bin/env python3

import math

import telemetry
from magicbot import MagicRobot, tunable
from rev import ServoHub, ServoHubConfig
from wpilib import CANPort, Field2d, Timer, XboxController
from wpimath import Rotation2d, Translation3d

from components.chassis import ChassisComponent
from components.vision import VisualLocalizer
from ids import DioChannel, ServoHubChannel
from swerves.potatobot import tuner_constants
from utilities.scalers import rescale_js
from utilities.servo import ServoCalibration, ServoTurret, TurretCalibration


class MyRobot(MagicRobot):
    chassis: ChassisComponent
    port_vision: VisualLocalizer

    # Driving constraints
    max_speed = tunable(0.8)  # m/s
    max_spin_rate = tunable(2.5)  # rad/s

    def create_objects(self) -> None:
        self.gamepad = XboxController(0)

        self.field = Field2d()

        self.servo_hub = ServoHub(CANPort.CAN_D0, 2)

        self.port_vision_name = "port_turret"
        self.port_vision_turret_pos = Translation3d(0.161, 0.169, 0.401)
        self.port_vision_turret_rot = Rotation2d()
        self.port_vision_camera_offset = Translation3d(0.027501, 0, 0.026724)
        self.port_vision_camera_pitch = math.radians(-5.0)
        self.port_vision_turret = ServoTurret(
            self.servo_hub.get_servo_channel(ServoHubChannel.PORT_VISION_TURRET),
            ServoCalibration(500, 1500, 2500, 55.0 / 60.0 * math.tau),
            DioChannel.PORT_VISION_ENCODER,
            TurretCalibration(
                encoder_at_forward=Rotation2d(2.055),
                encoder_at_center_pulse=Rotation2d(1.928),
                encoder_at_max_pulse=Rotation2d(3.960),
                encoder_at_min_limit=Rotation2d(0.952),
                encoder_at_max_limit=Rotation2d(3.482),
            ),
        )

        self.servo_hub.configure(
            ServoHubConfig().apply(
                ServoHubChannel.PORT_VISION_TURRET,
                self.port_vision_turret.channel_config(),
            )
        )

        self.chassis_tuner_constants = tuner_constants

        self.sim_loop_timer = Timer()
        self.sim_loop_timer.start()

    def robot_periodic(self) -> None:
        telemetry.log("Field", self.field)
        self.port_vision._per_loop_cache.clear()  # pyright: ignore[reportPrivateUsage]

    def disabled_periodic(self) -> None:
        self.chassis.update_alliance()
        self.chassis.update_odometry()
        self.port_vision.execute()

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

    def simulation_init(self) -> None:
        self.sim_loop_timer.reset()

    def simulation_periodic(self) -> None:
        tm_diff = self.sim_loop_timer.get()
        self.sim_loop_timer.reset()

        self.chassis.sim_periodic(tm_diff)
