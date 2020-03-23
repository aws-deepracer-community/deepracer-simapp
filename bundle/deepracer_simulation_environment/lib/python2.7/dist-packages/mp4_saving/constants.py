'''This module houses the constants for scripts package in simulation application'''
from enum import Enum
from markov.reset.constants import RaceType

class CameraTypeParams(Enum):
    """ This Enum contains the all the params for each camera topics
    Extends:
        Enum
    Variables:
        PIP_PARAMS (dict): picture in picture related information
        MAIN_CAMERA_PARAMS (dict): Main camera i.e, the camera view following the car
        SUB_CAMERA_PARAMS (dict): Sub camera i.e, the top view of the track
    """
    CAMERA_PIP_PARAMS = "camera_pip_params"
    CAMERA_45DEGREE_PARAMS = "camera_45degree_params"
    CAMERA_TOPVIEW_PARAMS = "camera_topview_params"

class Mp4Parameter(Enum):
    """
    Describes the parameters used to save Mp4
    Extends:
        Enum
    """
    FOURCC = 0x00000021
    FPS = 15
    FRAME_SIZE = (640, 480)

class RaceCarColorToRGB(Enum):
    """ Color to RGB mapping
    Extends:
        Enum
    """
    Black = (26, 26, 26)
    Grey = (135, 149, 150)
    Blue = (68, 95, 229)
    Red = (224, 26, 37)
    Orange = (255, 160, 10)
    White = (255, 255, 255)
    Purple = (159, 42, 195)

# Amount of time (in seconds) to wait, in order to prevent model state from
# spamming logs while the model is loading
WAIT_TO_PREVENT_SPAM = 2

# Radius of the circle plotted on the agent in pixels
RACECAR_CIRCLE_RADIUS = 30

# Mapping the racetype to the text shown on the video
RACE_TYPE_TO_VIDEO_TEXT_MAPPING = {
    RaceType.TIME_TRIAL.value: "Time trial",
    RaceType.OBJECT_AVOIDANCE.value: "Object avoidance",
    RaceType.HEAD_TO_BOT.value: "Head-to-bot",
    RaceType.HEAD_TO_MODEL.value: "Head-to-head"
}

# Decrease the minor image by a scale of provided number
SCALE_RATIO = 2.5

# Image size
class IconographicImageSize(Enum):
    """ The images that are provided are not accurate.
    Also some of the image pixels requires even number for the math.

    Attributes:
        FULL_IMAGE_SIZE: This is the same size that is given out by camera
        BOT_CAR_IMAGE_SIZE: Making the size rectangular and even pixel. Enlarging it.
        OBSTACLE_IMAGE_SIZE: [description]
        AGENTS_IMAGE_SIZE: [description]
        RACE_COMPLETE_IMAGE_SIZE: [description]
    """
    FULL_IMAGE_SIZE = (640, 480)
    BOT_CAR_IMAGE_SIZE = (34, 34)
    OBSTACLE_IMAGE_SIZE = (34, 34)
    AGENTS_IMAGE_SIZE = (88, 88)
    RACE_COMPLETE_IMAGE_SIZE = (308, 40)

# Track iconography png enums
class TrackAssetsIconographicPngs(Enum):
    """ Track images enum mapping

    Attributes:
        AGENTS_PNG: Different agents color images
        BOTS_PNG: Image of the bot shown in graphinology
        OBSTACLES_PNG: Image of the obstacle shown in graphinology
        OBSTACLE_OVERLAY_PNG: Gradient for the obstacle
        HEAD_TO_HEAD_OVERLAY_PNG: Gradient for the head to head
        RACE_COMPLETE_OVERLAY_PNG: Shown when the race is complete
    """
    AGENTS_PNG = ["DRL_video_racer1", "DRL_video_racer2"]
    BOTS_PNG = "DRL_video_bot"
    OBSTACLES_PNG = "DRL_video_obstacles"
    OBSTACLE_OVERLAY_PNG = "DRL_video_oa_overlay"
    HEAD_TO_HEAD_OVERLAY_PNG = "DRL_video_h2h_overlay"
    RACE_COMPLETE_OVERLAY_PNG = "DRL_video_racecomplete_overlay"
