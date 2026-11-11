import math
from logging import Logger

import ntcore
import wpilib
from magicbot import feedback
from phoenix6.swerve import requests
from phoenix6.swerve.swerve_module import SwerveModule
from wpimath import (
    ChassisVelocities,
    Pose2d,
    Rotation2d,
    SwerveModulePosition,
    SwerveModuleVelocity,
)
from wpimath.units import seconds

from swerves.drivetrain import TunerSwerveDrivetrain
from swerves.tuner_constants import TunerConstants
from utilities.game import is_red
from utilities.position import TeamPoses


class ChassisComponent:
    field: wpilib.Field2d
    logger: Logger

    LINEAR_ODOMETRY_STD_DEVS = 0.003
    ROTATION_ODOMETRY_STD_DEVS = 0.05

    def __init__(self, tuner_constants: TunerConstants) -> None:
        self.on_red_alliance = is_red()

        self.phoenix_swerve = TunerSwerveDrivetrain(
            tuner_constants.drivetrain_constants,
            [
                tuner_constants.front_left,
                tuner_constants.front_right,
                tuner_constants.back_left,
                tuner_constants.back_right,
            ],
        )

        self.phoenix_swerve.set_state_std_devs(
            (
                ChassisComponent.LINEAR_ODOMETRY_STD_DEVS,
                ChassisComponent.LINEAR_ODOMETRY_STD_DEVS,
                ChassisComponent.ROTATION_ODOMETRY_STD_DEVS,
            )
        )

        self.imu = self.phoenix_swerve.pigeon2

        self.field_request = requests.FieldCentric().with_forward_perspective(
            requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
        )

        self.robot_request = requests.RobotCentric()

        self.request = requests.Idle()

        self.initialise_telemetry()

    def setup(self) -> None:
        self.field_obj = self.field.get_object("fused_pose")
        self._initialise_pose()

    def _initialise_pose(self) -> None:
        self.set_pose(
            TeamPoses.RED_TEST_POSE
            if self.on_red_alliance
            else TeamPoses.BLUE_TEST_POSE
        )
        self.phoenix_swerve.set_operator_forward_direction(
            Rotation2d.from_degrees(180) if self.on_red_alliance else Rotation2d()
        )

    def update_alliance(self) -> None:
        # Check whether our alliance has "changed"
        # If so, it means we have an update from the FMS and need to re-init the odom
        if is_red() != self.on_red_alliance:
            self.on_red_alliance = is_red()
            self._initialise_pose()

    def on_enable(self) -> None:
        self.update_odometry()

    @feedback
    def get_pose(self) -> Pose2d:
        return self.phoenix_swerve.get_state().pose

    @feedback
    def get_imu_rotation(self) -> Rotation2d:
        return self.imu.get_rotation2d()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    @feedback
    def get_velocity(self) -> ChassisVelocities:
        return self.phoenix_swerve.get_state().velocity

    def reset_yaw(self) -> None:
        """Sets pose to current pose but with a heading of forwards"""
        cur_pose = self.get_pose()
        self.set_pose(
            Pose2d(
                cur_pose.translation(),
                Rotation2d(math.pi if self.on_red_alliance else 0),
            )
        )

    def set_pose(self, pose: Pose2d) -> None:
        self.phoenix_swerve.reset_pose(pose)
        self.field.set_robot_pose(pose)
        self.field_obj.set_pose(pose)

    def update_odometry(self) -> None:
        drivetrain_state = self.phoenix_swerve.get_state()
        self.field_obj.set_pose(drivetrain_state.pose)
        self.telemeterise(drivetrain_state)

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        self.request = self.prepare_request(self.field_request, vx, vy, omega)

    def drive_robot(self, vx: float, vy: float, omega: float) -> None:
        self.request = self.prepare_request(self.robot_request, vx, vy, omega)

    def stop(self) -> None:
        self.request = self.prepare_request(self.robot_request, 0.0, 0.0, 0.0)

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: seconds,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        self.phoenix_swerve.add_vision_measurement(
            vision_robot_pose,
            timestamp,
            vision_measurement_std_devs,
        )

    def prepare_request(
        self,
        request: requests.FieldCentric | requests.RobotCentric,
        vx: float,
        vy: float,
        omega: float,
    ) -> requests.SwerveRequest:
        return (
            request.with_velocity_x(vx)
            .with_velocity_y(vy)
            .with_rotational_rate(omega)
            .with_drive_request_type(SwerveModule.DriveRequestType.VELOCITY)
        )

    def execute(self) -> None:
        self.phoenix_swerve.set_control(self.request)
        self.request = (
            requests.Idle()
        )  # Safety so that robot stops if not commanded next cycle

        self.update_odometry()

    def sim_periodic(self, tm_diff: seconds) -> None:
        self.phoenix_swerve.update_sim_state(
            tm_diff, wpilib.RobotController.get_battery_voltage()
        )

        self.field.set_robot_pose(
            self.field.get_robot_pose().transform_by(
                self.get_velocity().to_twist2d(tm_diff).exp()
            )
        )

    def initialise_telemetry(self) -> None:
        nt = ntcore.NetworkTableInstance.get_default().get_table("/components/chassis")
        self.drive_state_table = nt.get_sub_table("module_states")
        self.drive_pose = self.drive_state_table.get_struct_topic(
            "Pose", Pose2d
        ).publish()
        self.drive_velocity = self.drive_state_table.get_struct_topic(
            "Velocity", ChassisVelocities
        ).publish()
        self.drive_module_velocities = self.drive_state_table.get_struct_array_topic(
            "ModuleVelocities", SwerveModuleVelocity
        ).publish()
        self.drive_module_targets = self.drive_state_table.get_struct_array_topic(
            "ModuleTargets", SwerveModuleVelocity
        ).publish()
        self.drive_module_positions = self.drive_state_table.get_struct_array_topic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self.drive_timestamp = self.drive_state_table.get_double_topic(
            "Timestamp"
        ).publish()
        self.drive_odometry_frequency = self.drive_state_table.get_double_topic(
            "OdometryFrequency"
        ).publish()

    def telemeterise(self, state: TunerSwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterise it to NetworkTables
        """
        # Telemeterise the swerve drive state
        self.drive_pose.set(state.pose)
        self.drive_velocity.set(state.velocity)
        self.drive_module_velocities.set(state.module_velocities)
        self.drive_module_targets.set(state.module_targets)
        self.drive_module_positions.set(state.module_positions)
        self.drive_timestamp.set(state.timestamp)
        self.drive_odometry_frequency.set(1.0 / state.odometry_period)
