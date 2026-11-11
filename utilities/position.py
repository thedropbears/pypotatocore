from wpimath import Pose2d

from utilities.game import field_flip_pose2d


class TeamPoses:
    RED_TEST_POSE = Pose2d(2.45, 5.50, 0)
    BLUE_TEST_POSE = field_flip_pose2d(RED_TEST_POSE)
