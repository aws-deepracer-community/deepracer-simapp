""" util module for all the mp4 saving
"""
import os
import logging
import numpy as np
import cv2
import rospkg
import rospy
from PIL import ImageFont, ImageDraw, Image
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.metrics.constants import (ITERATION_DATA_LOCAL_FILE_PATH,
                                      IterationDataLocalFileNames)
from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                         SIMAPP_SIMULATION_SAVE_TO_MP4_EXCEPTION)
from mp4_saving.constants import (RACECAR_CIRCLE_RADIUS, CameraTypeParams,
                                  SCALE_RATIO, IconographicImageSize)
from deepracer_simulation_environment.srv import TopCamDataSrvRequest, TopCamDataSrv

LOG = Logger(__name__, logging.INFO).get_logger()

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
    return get_image(track_name, IconographicImageSize.FULL_IMAGE_SIZE.value)

def get_image(icon_name, img_size=None):
    """ Given the icon_name in the track_iconography folder without png, gives back cv2 image
    with all 4 channels
    Args:
        icon_name (str): The name of the icon in the track_iconography folder
        img_size (tuple): If you want to resize the image (width, height) (default: {None})
    Returns:
        Image: The cv2 image read from the .png file
    """
    try:
        track_iconography_dir = os.path.join(
            rospkg.RosPack().get_path('deepracer_simulation_environment'), 'track_iconography')
        image_path = os.path.join(track_iconography_dir, icon_name + '.png')
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        if img_size:
            image = cv2.resize(image, img_size)
        return image
    except (OSError, IOError, Exception) as err_msg:
        log_and_exit("Iconography image does not exists or corrupt image: {}"
                         .format(err_msg),
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
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil_im = Image.fromarray(image)
    draw = ImageDraw.Draw(pil_im)
    draw_shadow(draw, text, font, loc[0], loc[1], font_shadow_color)
    draw.text(loc, text, font=font, fill=font_color)
    return cv2.cvtColor(np.array(pil_im), cv2.COLOR_RGB2BGR)

def create_folder_path(camera_dir_list):
    """ Create directory if the folder path does not exist
    Arguments:
        camera_dir_list (list): List of folder paths
    """
    for path in camera_dir_list:
        dir_path = os.path.dirname(path)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

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

def get_cameratype_params(racecar_name, agent_name):
    """ The function initializes the camera information for different camera settings
    This holds the location where the different camera mp4 are saved, also the topic
    of the camera to subscribe to get the videos.
    For the head to head racing, the top view camera is used to create graphics such
    that the bigger circles are drawn on the agent. So it can be easily identified which
    car is winning.

    Arguments:
        racecar_name (str): Name of the racecar
        agent_name (str): Agent name of the racecar
    Returns:
        (dict): camera information local path, topic, name
    """
    camera_info = dict()
    camera_pip_path = os.path.join(ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                   IterationDataLocalFileNames.CAMERA_PIP_MP4_VALIDATION_LOCAL_PATH.value)
    camera_45degree_path = os.path.join(ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                        IterationDataLocalFileNames.CAMERA_45DEGREE_MP4_VALIDATION_LOCAL_PATH.value)
    camera_topview_path = os.path.join(ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                       IterationDataLocalFileNames.CAMERA_TOPVIEW_MP4_VALIDATION_LOCAL_PATH.value)
    create_folder_path([camera_pip_path, camera_45degree_path, camera_topview_path])

    camera_info[CameraTypeParams.CAMERA_PIP_PARAMS] = {
        'name': CameraTypeParams.CAMERA_PIP_PARAMS.value,
        'topic_name': "/{}/deepracer/kvs_stream".format(racecar_name),
        'local_path': camera_pip_path
    }
    camera_info[CameraTypeParams.CAMERA_45DEGREE_PARAMS] = {
        'name': CameraTypeParams.CAMERA_45DEGREE_PARAMS.value,
        'topic_name': "/{}/main_camera/zed/rgb/image_rect_color".format(racecar_name),
        'local_path': camera_45degree_path
    }
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

def overlay_minor_img_to_bottom_right(major_cv_image, minor_cv_image):
    """ Given two images overlay minor image on the bottom right corner of the major image
    Args:
        major_cv_image (Image): Major image, usually the image that takes the complete screen
        minor_cv_image (Image): This will the image that will be scaled down and put at bottom right
    Returns:
        Image: The overlayed image is returned
    """
    # overlay minor on the bottom right of major
    minor_cv_image = resize_image(minor_cv_image, SCALE_RATIO)
    major_cv_image[-minor_cv_image.shape[0]:,
                   -minor_cv_image.shape[1]:] = minor_cv_image
    return major_cv_image

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

    try:
        alpha_foreground = rect_image[y_min_rect:y_max_rect, x_min_rect:x_max_rect, 3:4] / 255.0
        background_image[y_min_bg:y_max_bg, x_min_bg:x_max_bg, :4] = \
            (1 - alpha_foreground) * (background_image[y_min_bg:y_max_bg, x_min_bg:x_max_bg, :4]) \
            + alpha_foreground * rect_image[y_min_rect:y_max_rect, x_min_rect:x_max_rect, :4]
    except Exception as err_msg:
        LOG.info("Failed to render for this pixel {}; Error msg: {}".format(pixel_xy, err_msg))
    return background_image

def apply_gradient(main_image, gradient_img, gradient_alpha):
    """ The gradient on the image is overlayed so that text looks visible and clear.
    This leaves a good effect on the image.
    Args:
        main_image (Image): The main image where gradient has to be applied
        gradient_img (Image): The gradient image
        gradient_alpha (Numpy.Array): The 4th channel, the alpha channel of the gradient
    Returns:
        Image: Gradient applied image
    """
    for channel in range(0, 4):
        main_image[-gradient_img.shape[0]:, -gradient_img.shape[1]:, channel] = \
            (gradient_alpha * gradient_img[:, :, channel]) + \
            (1 - gradient_alpha) * (main_image[\
                -gradient_img.shape[0]:, -gradient_img.shape[1]:, channel])
    return main_image

def overlay_track_images(major_cv_image, minor_cv_image, loc_offset=(0, 0)):
    """ Overlaying the main image with the track iconographic image.
    Args:
        major_cv_image (Image): The main background image
        minor_cv_image (Image): Edited track iconographic image
        track_icongraphy_alpha (Numpy.Array): The 4th channel, the alpha channel of the iconographic image
    Returns:
        Image: One image with track overlayed on the main image
    """
    # Now adjust the alpha values to all channels
    minor_cv_image = resize_image(minor_cv_image, SCALE_RATIO)
    track_icongraphy_alpha = minor_cv_image[:, :, 3]/255.0

    # Minor image is placed at the bottom right with some offset
    x_min = -(loc_offset[1]+minor_cv_image.shape[0])
    x_max = major_cv_image.shape[0] - loc_offset[1]
    y_min = -(loc_offset[0]+minor_cv_image.shape[1])
    y_max = major_cv_image.shape[1] - loc_offset[0]

    for channel in range(0, 3):
        major_cv_image[x_min:x_max, y_min:y_max, channel] =\
            (track_icongraphy_alpha * minor_cv_image[:, :, channel]) + \
            (1 - track_icongraphy_alpha) * (major_cv_image[x_min:x_max, y_min:y_max, channel])
    return cv2.cvtColor(major_cv_image, cv2.COLOR_RGBA2RGB)
