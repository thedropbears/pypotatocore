import wpilib
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from wpimath import Pose2d, Transform3d, units

from components.vision import VisualLocalizer
from simulation.servo import ServoTurretSim
from utilities.game import apriltag_layout


class LocalizerSim:
    CAMERA_MAX_SIGHT_RANGE = 5.0  # m

    def __init__(self, world: VisionSystemSim, localizer: VisualLocalizer) -> None:
        self.world = world
        self.localizer = localizer

        self.turret_sim = ServoTurretSim(
            localizer.turret,
            localizer.turret.servo_calibration.max_speed,
            localizer.turret.neutral_angle,
        )

        self.camera_sim = PhotonCameraSim(
            localizer.camera, SimCameraProperties.OV9281_1280_720()
        )
        self.camera_sim.setMaxSightRange(self.CAMERA_MAX_SIGHT_RANGE)
        world.addCamera(self.camera_sim, self._robot_to_camera())
        self.camera_sim.enableRawStream(True)
        self.camera_sim.enableDrawWireframe(True)

    def update(self, tm_diff: units.seconds) -> None:
        # Move the turret first, so the camera is placed where it is this loop.
        self.turret_sim.update(tm_diff)
        # Track the camera every loop: the world keeps a history of camera
        # transforms and uses it to place latency-compensated frames.
        self.world.adjustCamera(self.camera_sim, self._robot_to_camera())

    def _robot_to_camera(self) -> Transform3d:
        return self.localizer.robot_to_camera(wpilib.Timer.get_monotonic_timestamp())


class VisionSim:
    RENDER_LOOP_DIVISOR = 10

    def __init__(self, localizers: list[VisualLocalizer]) -> None:
        self.world = VisionSystemSim("main")
        self.world.addAprilTags(apriltag_layout)
        self.localizer_sims = [LocalizerSim(self.world, loc) for loc in localizers]
        self._loop_counter = 0

    def update(self, tm_diff: units.seconds, true_pose: Pose2d) -> None:
        for localizer_sim in self.localizer_sims:
            localizer_sim.update(tm_diff)

        self._loop_counter += 1
        if self._loop_counter >= self.RENDER_LOOP_DIVISOR:
            self._loop_counter = 0
            self.world.update(true_pose)
