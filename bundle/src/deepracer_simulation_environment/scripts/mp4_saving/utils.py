#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

""" util module for all the mp4 saving
"""
import errno
import os
import logging
import numpy as np
import cv2
import rospkg
import rospy
from PIL import ImageFont, ImageDraw, Image
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION)
from mp4_saving.constants import (RACECAR_CIRCLE_RADIUS, CameraTypeParams,
                                  IconographicImageSize, SCALE_RATIO,
                                  TrackColors,
                                  SECTOR_COLORS_DICT)
from deepracer_simulation_environment.srv import TopCamDataSrvRequest, TopCamDataSrv

LOG = Logger(__name__, logging.INFO).get_logger()

CUSTOM_FILES_PATH = "./custom_files"
CAMERA_PIP_MP4_LOCAL_PATH_FORMAT = os.path.join(CUSTOM_FILES_PATH,
                                                "iteration_data/{}/camera-pip/video.mp4")
CAMERA_45DEGREE_LOCAL_PATH_FORMAT = os.path.join(CUSTOM_FILES_PATH,
                                                 "iteration_data/{}/camera-45degree/video.mp4")
CAMERA_TOPVIEW_LOCAL_PATH_FORMAT = os.path.join(CUSTOM_FILES_PATH,
                                                "iteration_data/{}/camera-topview/video.mp4")

IMAGE_CACHE = dict()

def plot_circle(image, x_pixel, y_pixel, color_rgb):
    """Function used to draw a circle around the racecar given the pixel (x, y) value
    and the color of the circle

    Args:
        image (Image): Top camera image
        x_pixel (int): X value in the image pixel
        y_pixel (int): Y value in the image pixel
        color_rgb (tuple): RGB value in the form of tuple (255, 255, 255)

    Returns:
        Image: Edited image with the circle
    """
    center_coordinates = (int(x_pixel), int(y_pixel))
    thickness = -1
    return cv2.circle(image, center_coordinates, RACECAR_CIRCLE_RADIUS, color_rgb, thickness)

def plot_rectangle(image, start_x_pixel, start_y_pixel, end_x_pixel, end_y_pixel, color_rgb):
    """Function used to draw a rectangle given the start and end pixel (x, y) values
    and the color of the rectangle

    Args:
        image (Image): Top camera image
        start_x_pixel (int): X value in the image start pixel
        start_y_pixel (int): Y value in the image start pixel
        end_x_pixel (int): X value in the image end pixel
        end_y_pixel (int): Y value in the image end pixel
        color_rgb (tuple): RGB value in the form of tuple (255, 255, 255)

    Returns:
        Image: Edited image with the rectangle
    """
    start_coordinates = (int(start_x_pixel), int(start_y_pixel))
    end_coordinates = (int(end_x_pixel), int(end_y_pixel))
    thickness = -1
    return cv2.rectangle(image, start_coordinates, end_coordinates, color_rgb, thickness)

def get_font(font_name, font_size):
    """Helper method that returns an ImageFont object for the desired font if
       available otherwise returns default font

    Args:
        font_name (str): String of the desired font
        font_size (str): Size of the font in points

    Returns:
        ImageFont: ImageFont object with the given font name
    """
    try:
        font_dir = os.path.join(rospkg.RosPack().get_path('deepracer_simulation_environment'),
                                'fonts')
        font_path = os.path.join(font_dir, font_name + '.ttf')
        font = ImageFont.truetype(font_path, font_size)
    except (OSError, IOError):
        LOG.info("%s unsupported font, using default font", font_name)
        font = ImageFont.load_default()
    return font

def get_track_iconography_image():
    """ The top camera image of the track is converted into icongraphic image
    making it look good. The size of the image kept the same as the image captured
    from the camera. The assumption here is the .png file is named same as track name

    Returns:
        Image: Reading the .png file as cv2 image
    """
    track_name = rospy.get_param("WORLD_NAME")
    track_img = get_image(track_name, IconographicImageSize.FULL_IMAGE_SIZE.value)
    return cv2.cvtColor(track_img, cv2.COLOR_RGBA2BGRA)

def get_image(icon_name, img_size=None, is_rgb=False):
    """ Given the icon_name in the track_iconography folder without png, gives back cv2 image
    with all 4 channels
    Args:
        icon_name (str): The name of the icon in the track_iconography folder
        img_size (tuple): If you want to resize the image (width, height) (default: {None})
    Returns:
        Image: The cv2 image read from the .png file
    """
    global IMAGE_CACHE
    if icon_name in IMAGE_CACHE:
        return IMAGE_CACHE[icon_name]
    try:
        track_iconography_dir = os.path.join(
            rospkg.RosPack().get_path('deepracer_simulation_environment'), 'track_iconography')
        image_path = os.path.join(track_iconography_dir, icon_name + '.png')
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
        if img_size:
            image = cv2.resize(image, img_size)
        IMAGE_CACHE[icon_name] = image
        return image
    except (OSError, IOError, Exception) as err_msg:
        log_and_exit("Iconography image does not exists or corrupt image: {}".format(err_msg),
                     SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)

def draw_shadow(draw_obj, text, font, x_loc, y_loc, shadowcolor):
    """Helper method that draws a shadow around given text for a given ImageDraw

    Args:
        draw_obj (ImageDraw): ImageDraw object where the shadow will drawn
        text (str): String to draw the shadow around
        font (ImageFont): The font of the string that the shadow will be drawn around
        x_loc (int): X location of the text string
        y_loc (int): Y location of text string
        shadowcolor (str): Color of the shadow
    """
    draw_obj.text((x_loc - 1, y_loc - 1), text, font=font, fill=shadowcolor)
    draw_obj.text((x_loc + 1, y_loc - 1), text, font=font, fill=shadowcolor)
    draw_obj.text((x_loc - 1, y_loc + 1), text, font=font, fill=shadowcolor)
    draw_obj.text((x_loc + 1, y_loc + 1), text, font=font, fill=shadowcolor)

def write_text_on_image(image, text, loc, font, font_color, font_shadow_color):
    """This function is used to write the text on the image using cv2 writer

    Args:
        image (Image): The image where the text should be written
        text (str): The actual text data to be written on the image
        loc (tuple): Pixel location (x, y) where the text has to be written
        font (ImageFont): The font style object
        font_color (tuple): RGB value of the font
        font_shadow_color (tuple): RGB color of the font shawdow

    Returns:
        Image: Edited image
    """
    pil_im = Image.fromarray(image)
    draw = ImageDraw.Draw(pil_im)
    draw_shadow(draw, text, font, loc[0], loc[1], font_shadow_color)
    draw.text(loc, text, font=font, fill=font_color)
    return np.array(pil_im)

def create_folder_path(camera_dir_list):
    """ Create directory if the folder path does not exist
    Arguments:
        camera_dir_list (list): List of folder paths
    """
    for path in camera_dir_list:
        dir_path = os.path.dirname(path)
        # addressing mkdir and check directory race condition:
        # https://stackoverflow.com/questions/12468022/python-fileexists-error-when-making-directory/30174982#30174982
        # TODO: change this to os.makedirs(simtrace_dirname, exist_ok=True) when we migrate off python 2.7
        try:
            os.makedirs(dir_path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
            LOG.error("File already exist %s", dir_path)

def milliseconds_to_timeformat(dtime_delta):
    """ Convert milliseconds to mm:ss.000 format
    Args:
        dt (datetime.timedelta): Datetime delta value
    Returns:
        (str): String in 00:00.000 time format (mm:ss.millisecond)
    """
    _, dt_hr_rem = divmod(dtime_delta.seconds, 3600)
    dt_min, dt_sec = divmod(dt_hr_rem, 60)
    dt_milli_sec = dtime_delta.microseconds // 1000
    return "{:02d}:{:02d}.{:03d}".format(dt_min, dt_sec, dt_milli_sec)

def get_speed_formatted_str(speed):
    """ Returns the speed with always two whole numbers and two decimal value.
    Example: 03.45
    Args:
        speed (float): The actual speed of the car
    Returns:
        str: The text format of the speed
    """
    speed_str = "{:0.2f}".format(round(speed, 2))
    return speed_str.zfill(5)


def get_cameratype_params(racecar_name, agent_name, is_f1_race_type):
    """ The function initializes the camera information for different camera settings
    This holds the location where the different camera mp4 are saved, also the topic
    of the camera to subscribe to get the videos.
    For the head to head racing, the top view camera is used to create graphics such
    that the bigger circles are drawn on the agent. So it can be easily identified which
    car is winning.

    Arguments:
        racecar_name (str): Name of the racecar
        agent_name (str): Agent name of the racecar
        is_f1_race_type (bool): Is this f1 race type
    Returns:
        (dict): camera information local path, topic, name
    """
    camera_info = dict()
    camera_pip_path = CAMERA_PIP_MP4_LOCAL_PATH_FORMAT.format(agent_name)
    camera_45degree_path = CAMERA_45DEGREE_LOCAL_PATH_FORMAT.format(agent_name)
    camera_topview_path = CAMERA_TOPVIEW_LOCAL_PATH_FORMAT.format(agent_name)
    create_folder_path([camera_pip_path, camera_45degree_path, camera_topview_path])

    camera_info[CameraTypeParams.CAMERA_PIP_PARAMS] = {
        'name': CameraTypeParams.CAMERA_PIP_PARAMS.value,
        'topic_name': "/{}/deepracer/main_camera_stream".format(racecar_name),
        'local_path': camera_pip_path
    }
    camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS] = {
        'name': CameraTypeParams.CAMERA_45DEGREE_PARAMS.value,
        'topic_name': "/{}/main_camera/zed/rgb/image_rect_color".format(racecar_name),
        'local_path': camera_45degree_path
    }
    # For F1 we have to edit the top camera image
    # %TODO editing is done for racecar_0, so all the other folders topcamera video file
    # will be corrupted since no frames will be published. Need to handle this case
    # gracefully both here and in markov package. Need changes to markov package because
    # it will try to upload a file that doesnt exist.
    if is_f1_race_type:
        camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS] = {
            'name': CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value,
            'topic_name': '/{}/topcamera/deepracer/mp4_stream'.format(racecar_name),
            'local_path': camera_topview_path
        }
    else:
        camera_info[CameraTypeParams.CAMERA_TOPVIEW_PARAMS] = {
            'name': CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value,
            'topic_name': "/sub_camera/zed/rgb/image_rect_color",
            'local_path': camera_topview_path
        }
    return camera_info

def resize_image(image, scale_ratio):
    """ Resize the image to a given scale
    Args:
        image (Image): Image that should be rescaled
        scale_ratio: The scale ratio of the image.
    Returns:
        Image: resized image as per the scale
    """
    rows, cols, _ = image.shape
    return cv2.resize(image, (int(cols//scale_ratio), int(rows//scale_ratio)))

def get_resized_alpha(image, scale_ratio):
    """ Resize the image to the scale specified and get alpha value
    Args:
        image (Image): Image that has to resized
    Returns:
        Numpy.Array: alpha value of the image
    """
    resized_img = resize_image(image, scale_ratio)
    # The 4th channel is the alpha channel. Normalize alpha channels from 0-255 to 0-1
    return resized_img[:, :, 3] / 255.0

def get_top_camera_info():
    """ Camera information is required to map the (x, y) value of the agent on the camera image.
    This is because each track will have its own FOV and padding percent because to avoid
    Z-fighting. Once the camera position, padding percentage is available. We can map the
    (x,y) of the agent with respect to the track to a new plane of the camera image with
    padding.

    Returns:
        dict: Camera information of top camera. FOV, padding, width, height image
    """
    rospy.wait_for_service('get_top_cam_data')
    top_camera_srv = ServiceProxyWrapper('get_top_cam_data', TopCamDataSrv)
    top_camera_info = top_camera_srv(TopCamDataSrvRequest())
    return top_camera_info

def plot_rectangular_image_on_main_image(background_image, rect_image, pixel_xy):
    """ This utility can be used when the icon with rectangular dimension and have
    even pixel. This is used to overlay icon on the background image. This handles
    the cases of the corners where the size of the icon goes out of the bound of
    the background image. The location where the icon image is provide (x, y). This
    will be considered as the center of the icon image

    Args:
        background_image (Image): Background image on which icon has to be drawn
        rect_image (Image): Icon image usually smaller image
        pixel_xy (tuple): Pixel (x, y) location on the background image

    Returns:
        Image: Overlayed image
    """
    try:
        x_offset = int(round(pixel_xy[0]))
        y_offset = int(round(pixel_xy[1]))

        # Compute the x/y corners in background image
        y_min_bg = y_offset - rect_image.shape[0]//2
        y_max_bg = y_min_bg + rect_image.shape[0]
        x_min_bg = x_offset - rect_image.shape[1]//2
        x_max_bg = x_min_bg + rect_image.shape[1]

        # Compute the x/y corners in rect image
        y_min_rect = 0
        y_max_rect = rect_image.shape[0]
        x_min_rect = 0
        x_max_rect = rect_image.shape[1]

        # Handle clipping around the edges
        y_clip_min = max(0, -y_min_bg)
        x_clip_min = max(0, -x_min_bg)
        y_clip_max = max(0, y_max_bg - background_image.shape[0])
        x_clip_max = max(0, x_max_bg - background_image.shape[1])

        y_min_bg += y_clip_min
        x_min_bg += x_clip_min
        y_min_rect += y_clip_min
        x_min_rect += x_clip_min
        y_max_bg -= y_clip_max
        x_max_bg -= x_clip_max
        y_max_rect -= y_clip_max
        x_max_rect -= x_clip_max

        alpha_foreground = rect_image[y_min_rect:y_max_rect, x_min_rect:x_max_rect, 3:4] / 255.0
        background_image[y_min_bg:y_max_bg, x_min_bg:x_max_bg, :4] = \
            (1 - alpha_foreground) * (background_image[y_min_bg:y_max_bg, x_min_bg:x_max_bg, :4]) \
            + alpha_foreground * rect_image[y_min_rect:y_max_rect, x_min_rect:x_max_rect, :4]
    except (Exception) as err_msg:
        #
        # This could fail when agents x, y location is NaN. This could when the markov node died.
        # This will also fail when the kinesis video node shuts down and the
        # main camera image is not received. Logging this pollutes the sim and one cannot look
        # at the actual errors. Hence removing the logger. Catching this exception so that it doesnot
        # bring down the Robomaker job.
        #
        pass
    return background_image


def get_gradient_values(gradient_img, multiplier=1):
    """ Given the image gradient returns gradient_alpha_rgb_mul and one_minus_gradient_alpha.
    These pre-calculated numbers are used to apply the gradient on the camera image

    Arguments:
        gradient_img (Image): Gradient image that has to applied on the camera image
        multiplier (float): This decides what percentage of gradient images alpha has to be applied.
                            This is useful in fading feature.

    Returns:
        (tuple): gradient_alpha_rgb_mul (Numpy.Array) gradient_img * gradient_alpha value
                 one_minus_gradient_alpha (Numpy.Array) (1 - gradient_alpha)
    """
    (height, width, _) = gradient_img.shape
    gradient_alpha = (gradient_img[:, :, 3] / 255.0 * multiplier).reshape(height, width, 1)

    gradient_alpha_rgb_mul = gradient_img * gradient_alpha
    one_minus_gradient_alpha = (1 - gradient_alpha).reshape(height, width)
    return gradient_alpha_rgb_mul, one_minus_gradient_alpha

def apply_gradient(main_image, gradient_alpha_rgb_mul, one_minus_gradient_alpha):
    """ The gradient on the image is overlayed so that text looks visible and clear.
    This leaves a good effect on the image.
    The older code took 6.348s for 1000 runs

    Numpy broadcasting is slower than normal python
    major_cv_image_1[:, :, :4] = (gradient_alpha_rgb_mul + (major_cv_image_1 * one_minus_gradient_alpha))[:, :, :4]
    Timeit 1000 runs - 6.523s

    The current code takes - 5.131s for 1000 runs

    Args:
        main_image (Image): The main image where gradient has to be applied
        gradient_alpha_rgb_mul (Numpy.Array): gradient_img * gradient_alpha value
        one_minus_gradient_alpha (Numpy.Array): (1 - gradient_alpha)
    Returns:
        Image: Gradient applied image
    """
    for channel in range(0, 4):
        main_image[:, :, channel] = gradient_alpha_rgb_mul[:, :, channel] + \
            (main_image[:, :, channel] * one_minus_gradient_alpha)
    return main_image

def racecar_name_to_agent_name(racecars_info, racecar_name):
    """ Given the racecars_info as list and the racecar_name and racecar_name
    get the agent name

    Arguments:
        racecars_info (list): List of racecars_info
        racecar_name (str): Racecar name
    """
    return 'agent' if len(racecars_info) == 1 else "agent_{}".format(racecar_name.split("_")[1])


def overlay_sector_color_on_track(major_cv_image, sector_img, x_min, x_max, y_min, y_max):
    """overlay the sector_img on top of major_cv_image at input location

    Args:
        major_cv_image (Image): major cv image as background
        sector_img (Image): overlay foreground cv image
        x_min (int): x min location for overlay foreground sector image
        x_max (int): x max location for overlay foreground sector image
        y_min (int): y min location for overlay foreground sector image
        y_max (int): y max location for overlay foreground sector image

    Returns:
        Image: combined image with foreground sector image on top of major cv image.
    """
    track_sector_icongraphy_scaled = resize_image(sector_img, SCALE_RATIO)
    track_sector_icongraphy_alpha = track_sector_icongraphy_scaled[:, :, 3] / 255.0

    for channel in range(0, 4):
        major_cv_image[x_min:x_max, y_min:y_max, channel] =\
            (track_sector_icongraphy_alpha * track_sector_icongraphy_scaled[:, :, channel]) + \
            (1 - track_sector_icongraphy_alpha) * (major_cv_image[x_min:x_max, y_min:y_max, channel])

    return major_cv_image


def get_sector_color(best_session_time, best_personal_time, current_personal_time):
    """compare the current personal sector time with best sector time and
    best personal sector time to determine sector overlay color by return the
    path to the image.

    Green for personal best, yellow for slower sectors, and purple for session best.

    Args:
        best_session_time (int): best session sector time in milliseconds
        best_personal_time (int): best personal sector time in milliseconds
        current_personal_time (int): current personal sector time in milliseconds

    Returns:
        str: Purple for session best, Green for personal best. Otherwise, yellow
    """

    # current personal time faster than best personal and best session time
    if current_personal_time <= best_personal_time and current_personal_time <= best_session_time:
        return TrackColors.PURPLE.value
    # current personal time faster than best personal only
    elif current_personal_time <= best_personal_time:
        return TrackColors.GREEN.value
    # current personal time is worse
    else:
        return TrackColors.YELLOW.value


def init_sector_img_dict(world_name, sector):
    """initialize sector color overlay images dictionary

    Args:
        world_name (str): current gazebo world name
        sector (str): sector name such as sector1 for example
    
    Returns:
        dict: sector img overlay dictionary with key as sector name + color name (sector1_yellow)
        and value as the Image.
    """
    sector_img_dict = dict()
    for color in TrackColors:
        sector_image = \
            get_image(world_name + SECTOR_COLORS_DICT[sector + "_" + color.value])
        sector_image = cv2.cvtColor(sector_image, cv2.COLOR_RGBA2BGRA)
        sector_img_dict[color.value] = sector_image
    return sector_img_dict
