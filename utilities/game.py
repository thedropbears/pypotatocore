"""Descriptions of the field and match state."""

import dataclasses
import typing

import robotpy_fields
import wpilib
from wpimath import Pose2d, Pose3d

apriltag_layout = robotpy_fields.get_field(
    robotpy_fields.FieldId.FRC_2026_REBUILT_WELDED
)

TagId = typing.Literal[
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
]

get_fiducial_pose = typing.cast(
    typing.Callable[[TagId], Pose3d],
    apriltag_layout.get_tag_pose,
)

APRILTAGS = apriltag_layout.get_tags()


@dataclasses.dataclass(slots=True)
class Tag2d:
    id: TagId
    pose: Pose2d


APRILTAGS_2D = [
    Tag2d(typing.cast(TagId, tag.id), tag.pose.to_pose2d()) for tag in APRILTAGS
]


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.MatchState.get_alliance() == wpilib.Alliance.RED
