"""Descriptions of the field and match state."""

import robotpy_fields
import wpilib
from wpimath import Pose2d, Rotation2d, Translation2d

field = robotpy_fields.get_field(robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED)

FIELD_LENGTH = field.get_length()
FIELD_WIDTH = field.get_width()


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, t.y)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.MatchState.get_alliance() == wpilib.Alliance.RED
