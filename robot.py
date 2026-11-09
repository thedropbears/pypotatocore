#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from phoenix6 import HootAutoReplay, HootReplay
#from phoenix6.swerve import SwerveModuleState

from robotcontainer import RobotContainer

import gtsam_apriltag
from photonlibpy import PhotonCamera
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim

import ntcore
import wpilib
import robotpy_fields
import wpimath
import telemetry


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robot_periodic which runs the scheduler for you
    """

    def __init__(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        super().__init__()

        self._replay = HootReplay("./logs/example.hoot")

        self._autonomous_command: commands2.Command | None = None
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer()

        # log and replay timestamp and Driver Station data
        self._time_and_ds_replay = (
            HootAutoReplay()
            .with_timestamp_replay()
            .with_driver_station_replay()
            .with_joystick_replay()
        )
        self.add_periodic(self._time_and_ds_replay.update, self.DEFAULT_PERIOD, -0.001)

        self.field = wpilib.Field2d()
        self.field_layout = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)
        self.localiser = self.field.get_object("localiser")

        self.camera = PhotonCamera("main")
        self.estimator = gtsam_apriltag.Estimator(self.field_layout, 2.0)
        self.estimator.enforce_field_limits(False)
        # self.estimator.set_floating_tag_ids([15, 16])
        # self.estimator.set_floating_tag_ids([id if id!=12 else 0 for id in range(32)])

        
        properties = SimCameraProperties.OV9281_1280_720()
        # properties = SimCameraProperties.PERFECT_90DEG()
        self.sim_camera = PhotonCameraSim(self.camera, properties)
        self.sim_camera.setMaxSightRange(5.0)        

        self.camera_transform = wpimath.Transform3d(0.0,0.0,1.50, wpimath.Rotation3d.from_degrees(0.0, 0.0, 0.0))

        nt = ntcore.NetworkTableInstance.get_default()
        photon_table = nt.get_table("photonvision")
        camera_table = photon_table.get_sub_table(self.camera.getName())
        self.camera_distortion_sub = camera_table.get_double_array_topic("cameraDistortion").subscribe([])
        self.camera_intrinsic_sub = camera_table.get_double_array_topic("cameraIntrinsics").subscribe([])
        self.camera_cal: None | gtsam_apriltag.CameraCalibration = None

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(self.field_layout)

        self.vision_sim.addCamera(
            self.sim_camera,
            self.camera_transform,
        )
        self._swerve_states = None

    def robot_periodic(self) -> None:
        state = self._container.drivetrain.get_state()
        pose = state.pose
        now = wpilib.Timer.get_timestamp()
        # print(self._container.drivetrain.get_state())
        self.field.set_robot_pose(pose)
        telemetry.log("Field", self.field)
        self.vision_sim.update(pose)

        if self.camera_cal is None:
            dist = self.camera_distortion_sub.get()
            intrinsics = self.camera_intrinsic_sub.get()
            if dist != [] and intrinsics != []:
                self.camera_cal = gtsam_apriltag.CameraCalibration(intrinsics, dist)

        if self.camera_cal is not None:
            result = self.camera.getLatestResult()
            for tag in result.getTargets():
                corners = [wpimath.Translation2d(c.x, c.y) for c in tag.detectedCorners]
                # obs = gtsam_apriltag.CornersObservation(now, tag.fiducialId,
                obs = gtsam_apriltag.CornersObservation(result.getTimestampSeconds(), tag.fiducialId,
                                                        corners, self.camera_cal, self.camera_transform)
                self.estimator.add_observation(obs)

        if self._swerve_states is None:
            self._swerve_states = state.module_positions.copy()
            self._last_imu = self._container.drivetrain.pigeon2.get_rotation2d()

        else:
            delta = self._container.drivetrain.kinematics.to_twist2d(self._swerve_states, state.module_positions)
            imu_r = self._container.drivetrain.pigeon2.get_rotation2d()
            imu_delta = imu_r - self._last_imu
            self._last_imu = imu_r
            self._swerve_states = state.module_positions.copy()
            self.estimator.update(imu_delta, delta, 0.02, now)
        self.localiser.set_pose(self.estimator.get_pose())


    def disabled_init(self) -> None:
        pass

    def disabled_periodic(self) -> None:
        pass

    def disabled_exit(self) -> None:
        pass

    def autonomous_init(self) -> None:
        # This autonomous runs the autonomous command selected by your RobotContainer class.
        self._autonomous_command = self._container.get_autonomous_command()

        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().schedule(self._autonomous_command)

    def autonomous_periodic(self) -> None:
        pass

    def autonomous_exit(self) -> None:
        pass

    def teleop_init(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self._autonomous_command:
            commands2.CommandScheduler.get_instance().cancel(self._autonomous_command)

    def teleop_periodic(self) -> None:
        pass

    def teleop_exit(self) -> None:
        pass

    def utility_init(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.get_instance().cancel_all()

    def utility_periodic(self) -> None:
        pass

    def utility_exit(self) -> None:
        pass
