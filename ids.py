import enum

from rev import ServoChannel


class ServoHubChannel:
    PORT_VISION_TURRET = ServoChannel.ChannelId.CHANNEL_ID0


class DioChannel(enum.IntEnum):
    PORT_VISION_ENCODER = 1
