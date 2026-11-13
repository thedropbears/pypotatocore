"""Descriptions of the field and match state."""

import wpilib


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.MatchState.get_alliance() == wpilib.Alliance.RED
