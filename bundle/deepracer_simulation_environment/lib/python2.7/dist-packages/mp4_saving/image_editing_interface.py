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
    def edit_image(self, major_cv_image):
        """ Function to edit the images
        Decorators:
            abc.abstractmethod
        Arguments:
            major_cv_image (Image): The 45degree camera image following the car
        Raises:
            NotImplementedError: Function to edit the image not implemented
        """
        raise NotImplementedError('Function to edit the image not implemented')
