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

'''This module contains the available sensors for the sim app'''
import logging
import time
import numpy as np

from sensor_msgs.msg import Image as sensor_image
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from PIL import Image
from markov.sensors.utils import get_observation_space, get_front_camera_embedders, \
                                 get_left_camera_embedders, get_stereo_camera_embedders, \
                                 get_lidar_embedders, get_observation_embedder
from markov.sensors.sensor_interface import SensorInterface, LidarInterface
from markov.environments.constants import TRAINING_IMAGE_SIZE
from markov.architecture.constants import Input
from markov.log_handler.deepracer_exceptions import GenericRolloutException, GenericError
from markov import utils
from markov.log_handler.logger import Logger
from markov.log_handler.constants import SIMAPP_SIMULATION_WORKER_EXCEPTION
from markov.rclpy_constants import SENSOR_ROS2_NODE_NAME
                                    

LOGGER = Logger(__name__, logging.INFO).get_logger()

# Common QoS profile for all sensor subscriptions
SENSOR_QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class SensorFactory:
    '''This class implements a sensot factory and is used to create sensors per
       agent.
    '''
    @staticmethod
    def create_sensor(racecar_name, sensor_type, config_dict):
        '''Factory method for creating sensors
            type - String containing the desired sensor type
            kwargs - Meta data, usually containing the topics to subscribe to, the
                     concrete sensor classes are responsible for checking the topics.
        '''
        
        if sensor_type == Input.CAMERA.value:
            return Camera(racecar_name)
        elif sensor_type == Input.LEFT_CAMERA.value:
            return LeftCamera(racecar_name)
        elif sensor_type == Input.STEREO.value:
            try:
                dual_camera = DualCamera(racecar_name)
                return dual_camera
            except Exception as ex:
                raise
        elif sensor_type == Input.LIDAR.value:
            return Lidar(racecar_name)
        elif sensor_type == Input.SECTOR_LIDAR.value:
            return SectorLidar(racecar_name)
        elif sensor_type == Input.DISCRETIZED_SECTOR_LIDAR.value:
            return DiscretizedSectorLidar(racecar_name, config_dict)
        elif sensor_type == Input.OBSERVATION.value:
            return Observation(racecar_name)
        else:
            raise GenericRolloutException("Unknown sensor")


class SensorSubscriber:
    """Base class for ROS2 sensor subscribers that uses the existing markov_scripts_node"""
    
    def __init__(self):
        """Initialize the sensor subscriber - no separate node creation needed"""
        # Subscribe to /clock topic to ensure proper timing synchronization
        try:
            from rosgraph_msgs.msg import Clock
            
            node = self.getNode()
            if node:
                self.clock_subscription = node.create_subscription(
                    Clock,
                    '/clock',
                    self._clock_callback,
                    10
                )
        except Exception as e:
            LOGGER.error(f"Error in sensor initialization: {e}")
    
    def _clock_callback(self, msg):
        """Callback for /clock topic - helps with timing synchronization"""
        pass
    
    def getNode(self):
        """Get the existing markov_scripts_node instead of creating a separate node"""
        try:
            import rclpy
            
            from markov.rclpy_wrappers import ROS2NodeManager
            
            # Use a global singleton to ensure we get the same instance
            if not hasattr(SensorSubscriber, '_node_manager') or SensorSubscriber._node_manager is None:
                SensorSubscriber._node_manager = ROS2NodeManager.get_instance(SENSOR_ROS2_NODE_NAME)
            else:
                LOGGER.debug("Node manager already exists")
            
            node = SensorSubscriber._node_manager.node
            
            return node
        except Exception as ex:
            raise GenericRolloutException("Failed to get ROS2 node: {}".format(ex))


class Camera(SensorInterface, SensorSubscriber):
    '''Single camera sensor'''
    def __init__(self, racecar_name,  timeout=120.0):
        """Camera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(Camera, self).__init__()
        self.image_buffer = utils.DoubleBuffer()
        
        camera_topic = f'/{racecar_name}/camera/zed/rgb/image_rect_color'
        
        # Add debugging and error handling for subscription creation
        try:
            
            self.subscription = self.getNode().create_subscription(
                sensor_image, 
                camera_topic, 
                self._camera_cb_, 
                SENSOR_QOS_PROFILE
            )
            
        except Exception as ex:
            raise GenericRolloutException("Failed to create camera subscription: {}".format(ex))
            
        self.raw_data = None
        self.sensor_type = Input.CAMERA.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(Input.CAMERA.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            if image_data is None:
                raise GenericRolloutException("Camera image_data is None - camera not available")
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.CAMERA.value: np.array(image)}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_front_camera_embedders(network_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class Observation(SensorInterface, SensorSubscriber):
    '''Single camera sensor that is compatible with simapp v1'''
    def __init__(self, racecar_name, timeout=120.0):
        """Observation sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(Observation, self).__init__()
        self.image_buffer = utils.DoubleBuffer()
        self.getNode().create_subscription(sensor_image, f'/{racecar_name}/camera/zed/rgb/image_rect_color', self._camera_cb_, 
                                          SENSOR_QOS_PROFILE)
        self.sensor_type = Input.OBSERVATION.value
        self.raw_data = None
        self.timeout = timeout
    
    def get_observation_space(self):
        try:
            return get_observation_space(Input.OBSERVATION.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            # Make sure the first image is the starting image
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            # Check if image_data is None
            if image_data is None:
                raise GenericRolloutException("Observation image_data is None - camera not available")
            # Read the image and resize to get the state
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.OBSERVATION.value: np.array(image)}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_observation_embedder()
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class LeftCamera(SensorInterface, SensorSubscriber):
    '''This class is specific to left camera's only, it used the same topic as
       the camera class but has a different observation space. If this changes in
       the future this class should be updated.
    '''
    def __init__(self, racecar_name, timeout=120.0):
        """LeftCamera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(LeftCamera, self).__init__()
        self.image_buffer = utils.DoubleBuffer()
        self.getNode().create_subscription(sensor_image, f'/{racecar_name}/camera/zed/rgb/image_rect_color', self._camera_cb_, 
                                          SENSOR_QOS_PROFILE)
        self.sensor_type = Input.LEFT_CAMERA.value
        self.raw_data = None
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(Input.LEFT_CAMERA.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            # Make sure the first image is the starting image
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            # Check if image_data is None
            if image_data is None:
                raise GenericRolloutException("LeftCamera image_data is None - camera not available")
            # Read the image and resize to get the state
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.LEFT_CAMERA.value: np.array(image)}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_left_camera_embedders(network_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class DualCamera(SensorInterface, SensorSubscriber):
    '''This class handles the data for dual cameras'''
    def __init__(self, racecar_name, timeout=120.0):
        """DualCamera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(DualCamera, self).__init__()

        # Queue used to maintain image consumption synchronicity
        self.image_buffer_left = utils.DoubleBuffer()
        self.image_buffer_right = utils.DoubleBuffer()

        # Set up the subscribers
        left_topic = f'/{racecar_name}/camera/zed/rgb/image_rect_color'
        right_topic = f'/{racecar_name}/camera/zed_right/rgb/image_rect_color_right'
        
        left_sub = self.getNode().create_subscription(sensor_image, left_topic, self._left_camera_cb_, 
                                                     SENSOR_QOS_PROFILE)
        right_sub = self.getNode().create_subscription(sensor_image, right_topic, self._right_camera_cb_, 
                                                      SENSOR_QOS_PROFILE)
        
        self.sensor_type = Input.STEREO.value
        self.timeout = timeout
        self.topics_ready = False
        self.left_topic = left_topic
        self.right_topic = right_topic

    def get_observation_space(self):
        try:
            return get_observation_space(Input.STEREO.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _wait_for_topics(self, max_wait_time=90.0):
        """Wait for camera topics to become available before trying to get images"""
        
        if self.topics_ready:
            return True
            
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            # Get list of available topics
            topic_names_and_types = self.getNode().get_topic_names_and_types()
            available_topics = [name for name, _ in topic_names_and_types]
            
            # Check if both camera topics are available
            left_available = self.left_topic in available_topics
            right_available = self.right_topic in available_topics
            
            if left_available and right_available:
                self.topics_ready = True
                return True
                
            # Wait a bit before checking again
            time.sleep(1.0)
            
        return False

    def get_state(self, block=True):
        
        # Wait for camera topics to become available first
        if not self._wait_for_topics():
            return {}
        
        left_img = None
        right_img = None
        
        # Single attempt - buffer timeout handles waiting
        try:
            left_image_data = self.image_buffer_left.get(block=block, timeout=self.timeout)
            if left_image_data is not None:
                left_img = Image.frombytes('RGB', (left_image_data.width, left_image_data.height),
                                           left_image_data.data, 'raw', 'RGB', 0, 1)
                left_img = left_img.resize(TRAINING_IMAGE_SIZE, resample=2).convert('L')
        except utils.DoubleBuffer.Empty:
            pass
        except Exception as ex:
            LOGGER.exception("Error getting left camera image: %s", ex)

        try:
            right_image_data = self.image_buffer_right.get(block=block, timeout=self.timeout)
            if right_image_data is not None:
                right_img = Image.frombytes('RGB', (right_image_data.width, right_image_data.height),
                                            right_image_data.data, 'raw', 'RGB', 0, 1)
                right_img = right_img.resize(TRAINING_IMAGE_SIZE, resample=2).convert('L')
        except utils.DoubleBuffer.Empty:
            pass
        except Exception as ex:
            LOGGER.exception("Error getting right camera image: %s", ex)

        # Check what we got
        if left_img is None and right_img is None:
            return {}
        elif left_img is None:
            return {}
        elif right_img is None:
            return {}
        else:
            # Both cameras OK
            try:
                result = {Input.STEREO.value: np.array(np.stack((left_img, right_img), axis=2))}
                return result
            except Exception as ex:
                return {}

    def reset(self):
        self.image_buffer_left.clear()
        self.image_buffer_right.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_stereo_camera_embedders(network_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _left_camera_cb_(self, data):
        ''' Callback for the left camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer_left.put(data)
        except Exception as ex:
            LOGGER.error("Error in left camera callback: %s", ex)

    def _right_camera_cb_(self, data):
        ''' Callback for the right camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer_right.put(data)
        except Exception as ex:
            LOGGER.error("Error in right camera callback: %s", ex)


class Lidar(LidarInterface, SensorSubscriber):
    '''This class handles the data collection for lidar'''
    def __init__(self, racecar_name, timeout=120.0):
        """Lidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(Lidar, self).__init__()

        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.getNode().create_subscription(LaserScan, f'/{racecar_name}/scan', self._scan_cb, 
                                          SENSOR_QOS_PROFILE)
        self.sensor_type = Input.LIDAR.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: self.data_buffer.get(block=block, timeout=self.timeout)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: self.data_buffer.get(block=True, timeout=self.timeout)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_lidar_embedders(network_type, self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.info("Unable to retrieve state: %s", ex)


class SectorLidar(LidarInterface, SensorSubscriber):
    '''This class handles the data collection for sector lidar'''
    def __init__(self, racecar_name, timeout=120.0):
        """SectorLidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(SectorLidar, self).__init__()

        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.getNode().create_subscription(LaserScan, f'/{racecar_name}/scan', self._scan_cb, 
                                          SENSOR_QOS_PROFILE)

        self.sensor_type = Input.SECTOR_LIDAR.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: self.data_buffer.get(block=block, timeout=self.timeout)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: self.data_buffer.get(block=True, timeout=self.timeout)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_lidar_embedders(network_type, self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.error("[LIDAR DEBUG] SectorLidar._scan_cb ERROR: %s", ex)

class DiscretizedSectorLidar(LidarInterface, SensorSubscriber):
    '''This class handles the data collection for sector lidar'''
    def __init__(self, racecar_name, config, timeout=120.0):
        """DiscretizedSectorLidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        super(DiscretizedSectorLidar, self).__init__()

        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        self.getNode().create_subscription(LaserScan, f'/{racecar_name}/scan', self._scan_cb, 
                                          SENSOR_QOS_PROFILE)
        self.sensor_type = Input.DISCRETIZED_SECTOR_LIDAR.value
        self.model_metadata = config["model_metadata"]
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type, self.model_metadata)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: self.data_buffer.get(block=block, timeout=self.timeout)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: self.data_buffer.get(block=True, timeout=self.timeout)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def get_input_embedders(self, network_type):
        try:
            return get_lidar_embedders(network_type, self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.info("Unable to retrieve state: %s", ex)
