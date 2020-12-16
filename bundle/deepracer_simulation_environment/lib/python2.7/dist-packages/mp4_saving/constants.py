'''This module houses the constants for scripts package in simulation application'''
from enum import Enum
from markov.reset.constants import RaceType
import cv2

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
    FOURCC = cv2.VideoWriter_fourcc(*'mp4v')
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

# AWS Deepracer watermark
AWS_DEEPRACER_WATER_MARK = "AWS DeepRacer"

# Mapping the racetype to the text shown on the video
RACE_TYPE_TO_VIDEO_TEXT_MAPPING = {
    RaceType.TIME_TRIAL.value: "Time trial",
    RaceType.OBJECT_AVOIDANCE.value: "Object avoidance",
    RaceType.HEAD_TO_BOT.value: "Head-to-bot",
    RaceType.HEAD_TO_MODEL.value: "Head-to-head",
    RaceType.F1.value: "F1"
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
    BOT_CAR_IMAGE_SIZE = (int(34//SCALE_RATIO), int(34//SCALE_RATIO))
    OBSTACLE_IMAGE_SIZE = (int(34//SCALE_RATIO), int(34//SCALE_RATIO))
    AGENTS_IMAGE_SIZE = (int(88//SCALE_RATIO), int(88//SCALE_RATIO))
    RACE_COMPLETE_IMAGE_SIZE = (308, 40)
    # F1 related
    F1_LOGO_IMAGE_SIZE = (125, 50)
    F1_RACER_SLASH_DISPLAY_ICON_SIZE = (35, 20)
    F1_RACER_RECT_DISPLAY_ICON_SIZE = (6, 13)
    F1_AGENTS_IMAGE_SIZE = (18, 18)

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
    HEAD_TO_HEAD_OVERLAY_PNG_LEAGUE_LEADERBOARD = "DRL_video_h2h_overlay_league_leaderboard"
    OBSTACLE_OVERLAY_PNG_LEAGUE_LEADERBOARD = "DRL_video_oa_overlay_league_leaderboard"
    #F1
    F1_LOGO_PNG = "F1_logo_official"
    F1_OVERLAY_DEFAULT_PNG = "F1_video_overlay_default"
    F1_OVERLAY_MIDWAY_PNG = "F1_video_overlay_midway"
    F1_OVERLAY_FINISHERS_PNG = "F1_video_overlay_finishers"
    F1_OVERLAY_TOPVIEW_2BOX_PNG = "F1_video_overlay_topview_2box"
    F1_OVERLAY_TOPVIEW_3BOX_PNG = "F1_video_overlay_topview_3box"
    F1_AGENTS_PNG = "./oval/oval"
    F1_AGENTS_NUM_PNG = "./numbers/number"
    F1_AGENTS_SLASH_DISPLAY_ICON_PNG = "./slash/slash"
    F1_AGENTS_RECT_DISPLAY_ICON_PNG = "./rectangle/rectangle"

class XYPixelLoc(Enum):
    """ The mp4 image size is (480, 640). Rendering text at different locations
    """
    MULTI_AGENT_DISPLAY_NAME_LOC = [(10, 10), (450, 10)]
    MULTI_AGENT_EVAL_TIME = (240, 10)
    SINGLE_AGENT_DISPLAY_NAME_LOC = (10, 10)
    SPEED_LEADERBOARD_LOC = (10, 410)
    SPEED_EVAL_LOC = (10, 420)
    LEADERBOARD_NAME_LOC = (10, 435)
    RACE_TYPE_EVAL_LOC = (10, 445)
    RACE_TYPE_RACE_LOC = (10, 455)
    AWS_DEEPRACER_WATER_MARK_LOC = (445, 450)
    TRAINING_PHASE_LOC = (40, 400)
    TRACK_IMG_WITH_OFFSET_LOC = (0, 20)
    TRACK_IMG_WITHOUT_OFFSET_LOC = (0, 0)
    # F1 related pixel
    F1_LOGO_LOC = (80, 35)
    F1_DISPLAY_NAME_SLASH_LOC = (40, 390)
    F1_DISPLAY_NAME_LOC = (60, 386)
    F1_RANKING_LOC = (20, 410)
    F1_LAP_EVAL_GAP_LOC = (20, 435)
    F1_LAP_EVAL_GAP_VAL_LOC = (270, 435)
    F1_LEADERBOARD_NAME_LOC = (20, 460)
    # F1 midway progress
    F1_MIDWAY_LAP_TEXT_LOC = (50, 55)
    F1_MIDWAY_LAP_COUNTER_LOC = (57, 73)
    F1_MIDWAY_LEADER_RANK_LOC = (30, 100)
    F1_MIDWAY_LEADER_COLOR_CODE_LOC = (55, 105)
    F1_MIDWAY_LEADER_DISPLAY_NAME_LOC = (62, 100)
    F1_MIDWAY_LEADER_GAP_LOC = (140, 100)
    # F1 finished lap
    F1_FINISHED_TITLE_LOC = (40, 120)
    F1_FINISHED_RANK_LOC = (30, 145)
    F1_FINISHED_COLOR_CODE_LOC = (50, 150)
    F1_FINISHED_DISPLAY_NAME_LOC = (62, 145)
    # F1 Top camera
    F1_TOP_CAMERA_LAP_TEXT_LOC = (10, 10)
    F1_TOP_CAMERA_LAP_COUNTER_LOC = (10, 30)
    F1_TOP_CAMERA_LEADER_RANK_LOC = (80, 15)
    F1_TOP_CAMERA_LEADER_COLOR_CODE_LOC = (100, 20)
    F1_TOP_CAMERA_LEADER_DISPLAY_NAME_LOC = (107, 15)
    F1_TOP_CAMERA_LEADER_GAP_LOC = (182, 15)
    # F1 Agent num offset
    F1_AGENT_NUM_OFFSET = (8, 8)

# Race completion flag y-offset
RACE_COMPLETE_Y_OFFSET = 180

# Mapping the racetype to the text shown on the video
class FrameQueueData(Enum):
    """ Enum for frame data put into the queue
    """
    FRAME = "frame",
    AGENT_METRIC_INFO = "agent_metric_info",
    TRAINING_PHASE = "training_phase"

# Agent Video editor constants
MAX_FRAMES_IN_QUEUE = 100
KVS_PUBLISH_PERIOD = 1.0/15.0
QUEUE_WAIT_TIME = 10 # In seconds


class ModelImgNames(Enum):
    """ Mapping the different objects and agents image

    Attributes:
        OBJECT_IMGS: This could be boxes or the bots
        AGENT_IMGS: This is the agents image
        AGENT_NUM_IMGS: This is the agent number, used only in F1
    """
    OBJECT_IMGS = "object_imgs"
    AGENT_IMGS = "agent_imgs"
    AGENT_NUM_IMGS = "agent_num_imgs"


class FrameTypes(Enum):
    """ Different frames to be edited as part of Mp4 and KVS

    Attributes:
        MAIN_CAMERA_FRAME: Frames coming from 45degree camera
        TOP_CAMERA_FRAME: Frames from the top view camera
    """
    MAIN_CAMERA_FRAME = "main_camera_frame"
    TOP_CAMERA_FRAME = "top_camera_frame"
