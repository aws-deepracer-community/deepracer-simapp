"""Interface class to edit the images for different race_type
"""
import abc

class ImageEditingInterface(object):
    """ Interface to edit the images for different race_type
    Extends:
        metaclass=abc.ABCMeta
    """
    __metaclass__ = abc.ABCMeta
    @abc.abstractmethod
    def edit_image(self, major_cv_image, metric_info):
        """ Function to edit the images
        Decorators:
            abc.abstractmethod
        Arguments:
            major_cv_image (Image): The 45degree camera image following the car
            metric_info (Dict): Dict of agent_metric_info(List of ROS metric values of each agent) and training_phase
        Raises:
            NotImplementedError: Function to edit the image not implemented
        """
        raise NotImplementedError('Function to edit the image not implemented')
