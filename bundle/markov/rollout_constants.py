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

from enum import Enum
from std_msgs.msg import ColorRGBA


class YamlKey(Enum):
    CAR_COLOR = 'CAR_COLOR'
    BODY_SHELL_TYPE = 'BODY_SHELL_TYPE'
    RACE_TYPE = 'RACE_TYPE'


F1 = 'f1'


class BodyShellType(Enum):
    DEFAULT = 'deepracer'
    F1_2021 = 'f1_2021'
    F1_CAR_11 = 'f1_car_11'
    F1_CAR_12 = 'f1_car_12'
    F1_CAR_13 = 'f1_car_13'
    F1_CAR_14 = 'f1_car_14'
    F1_CAR_15 = 'f1_car_15'
    F1_CAR_16 = 'f1_car_16'
    F1_CAR_17 = 'f1_car_17'
    F1_CAR_18 = 'f1_car_18'
    F1_CAR_19 = 'f1_car_19'
    F1_CAR_20 = 'f1_car_20'
    F1_CAR_21 = 'f1_car_21'


DEFAULT_COLOR = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)


class CarColorType(Enum):
    BLACK = 'Black'
    GREY = 'Grey'
    BLUE = 'Blue'
    RED = 'Red'
    ORANGE = 'Orange'
    WHITE = 'White'
    PURPLE = 'Purple'
    F1_CAR_1E73FC = 'f1_car_1e73fc'
    F1_CAR_7102BB = 'f1_car_7102bb'
    F1_CAR_CF15FC = 'f1_car_cf15fc'
    F1_CAR_EB0000 = 'f1_car_eb0000'
    F1_CAR_FF9900 = 'f1_car_ff9900'
    F1_CAR_FFFE01 = 'f1_car_fffe01'
    F1_CAR_9EFC03 = 'f1_car_9efc03'
    F1_CAR_1BD900 = 'f1_car_1bd900'
    F1_CAR_73FDF9 = 'f1_car_73fdf9'
    F1_CAR_000000 = 'f1_car_000000'
    F1_CAR_BCCACC = 'f1_car_bccacc'
    F1_CAR_FFFFFF = 'f1_car_ffffff'


COLOR_MAP = {
    CarColorType.BLACK.value: ColorRGBA(r=.1, g=0.1, b=0.1, a=1.0),
    CarColorType.GREY.value: ColorRGBA(r=0.529, g=0.584, b=0.588, a=1.0),
    CarColorType.BLUE.value: ColorRGBA(r=0.266, g=0.372, b=0.898, a=1.0),
    CarColorType.RED.value: ColorRGBA(r=0.878, g=0.101, b=0.145, a=1.0),
    CarColorType.ORANGE.value: ColorRGBA(r=1.0, g=0.627, b=0.039, a=1.0),
    CarColorType.WHITE.value: ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
    CarColorType.PURPLE.value: ColorRGBA(r=0.611, g=0.164, b=0.764, a=1.0),
    CarColorType.F1_CAR_1E73FC.value: ColorRGBA(r=0.117, g=0.450, b=0.988, a=1.0),
    CarColorType.F1_CAR_7102BB.value: ColorRGBA(r=0.443, g=0.007, b=0.733, a=1.0),
    CarColorType.F1_CAR_CF15FC.value: ColorRGBA(r=0.811, g=0.082, b=0.988, a=1.0),
    CarColorType.F1_CAR_EB0000.value: ColorRGBA(r=0.921, g=0.000, b=0.000, a=1.0),
    CarColorType.F1_CAR_FF9900.value: ColorRGBA(r=1.000, g=0.600, b=0.000, a=1.0),
    CarColorType.F1_CAR_FFFE01.value: ColorRGBA(r=1.000, g=0.996, b=0.003, a=1.0),
    CarColorType.F1_CAR_9EFC03.value: ColorRGBA(r=0.619, g=0.988, b=0.011, a=1.0),
    CarColorType.F1_CAR_1BD900.value: ColorRGBA(r=0.105, g=0.850, b=0.000, a=1.0),
    CarColorType.F1_CAR_73FDF9.value: ColorRGBA(r=0.450, g=0.992, b=0.976, a=1.0),
    CarColorType.F1_CAR_000000.value: ColorRGBA(r=0.000, g=0.000, b=0.000, a=1.0),
    CarColorType.F1_CAR_BCCACC.value: ColorRGBA(r=0.737, g=0.792, b=0.800, a=1.0),
    CarColorType.F1_CAR_FFFFFF.value: ColorRGBA(r=1.000, g=1.000, b=1.000, a=1.0)
}
