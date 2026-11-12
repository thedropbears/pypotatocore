from typing import override

import commands3
import wpilib
from phoenix6 import HootAutoReplay, HootReplay

from robotcontainer import RobotContainer


class MyRobot(wpilib.OpModeRobot):
    def __init__(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        super().__init__()

        self._replay = HootReplay("./logs/example.hoot")

        self._container = RobotContainer()
        self.drivetrain = self._container.drivetrain

        # log and replay timestamp and Driver Station data
        self._time_and_ds_replay = (
            HootAutoReplay()
            .with_timestamp_replay()
            .with_driver_station_replay()
            .with_joystick_replay()
        )
        self.add_periodic(self._time_and_ds_replay.update, self.DEFAULT_PERIOD)

    @override
    def robot_periodic(self) -> None:
        self._container.periodic()
        commands3.Scheduler.get_default().run()

    @override
    def disabled_init(self) -> None:
        pass

    @override
    def disabled_periodic(self) -> None:
        self._container.disabled_periodic()

    @override
    def disabled_exit(self) -> None:
        pass
