from wpimath import Pose2d

from utilities.game import field_flip_pose2d


class TeamPoses:
    BLUE_TEST_POSE = Pose2d(2.45, 5.50, 0)
    RED_TEST_POSE = field_flip_pose2d(BLUE_TEST_POSE)
