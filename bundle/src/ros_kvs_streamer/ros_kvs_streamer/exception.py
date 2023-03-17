class KvsException(Exception):
    """
    KvsException class
    """
    def __init__(self, message="", error_code=500):
        """
        Initialize KvsException

        Args:
            message (str): message
            error_code (int): error code
        """
        self.error_code = error_code
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return "{} ({})".format(self.message, self.error_code)


class KvsError(KvsException):
    """
    KvsError class
    """
    def __init__(self, message="", error_code=400):
        """
        Initialize KvsError

        Args:
            message (str): message
            error_code (int): error code
        """
        super().__init__(error_code=error_code,
                         message=message)
