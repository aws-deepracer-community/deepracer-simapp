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

""" The script takes care of testing the functionality of test_deepracer_exceptions.py
"""
import pytest
from markov.log_handler.deepracer_exceptions import (RewardFunctionError, GenericRolloutException,
                                                     GenericTrainerException, GenericTrainerError,
                                                     GenericRolloutError, GenericValidatorException,
                                                     GenericValidatorError, GenericException,
                                                     GenericError)

@pytest.mark.robomaker
@pytest.mark.sagemaker
def test_deepracer_exceptions():
    """The function tests whether the user defined exceptions in deepracer_exceptions.py are
    getting raised properly when we call them from any part of SIMAPP code.

    The test function also checks whether the superclass Exception manages to provide
    the necessary error message passed along as well.

    Raises:
        RewardFunctionError
        GenericTrainerException
        GenericTrainerError
        GenericRolloutException
        GenericRolloutError
        GenericValidatorException
        GenericValidatorError
        GenericException
        GenericError
    """
    with pytest.raises(RewardFunctionError, match=r".*RewardFunctionError.*"):
        raise RewardFunctionError("RewardFunctionError")
    with pytest.raises(GenericTrainerException, match=r".*GenericTrainerException.*"):
        raise GenericTrainerException("GenericTrainerException")
    with pytest.raises(GenericTrainerError, match=r".*GenericTrainerError.*"):
        raise GenericTrainerError("GenericTrainerError")
    with pytest.raises(GenericRolloutException, match=r".*GenericRolloutException.*"):
        raise GenericRolloutException("GenericRolloutException")
    with pytest.raises(GenericRolloutError, match=r".*GenericRolloutError.*"):
        raise GenericRolloutError("GenericRolloutError")
    with pytest.raises(GenericValidatorException, match=r".*GenericValidatorException.*"):
        raise GenericValidatorException("GenericValidatorException")
    with pytest.raises(GenericValidatorError, match=r".*GenericValidatorError.*"):
        raise GenericValidatorError("GenericValidatorError")
    with pytest.raises(GenericException, match=r".*GenericException.*"):
        raise GenericException("GenericException")
    with pytest.raises(GenericError, match=r".*GenericError.*"):
        raise GenericError("GenericError")
