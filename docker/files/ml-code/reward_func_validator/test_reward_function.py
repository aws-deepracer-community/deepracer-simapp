# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import sys

sys.path.append(
    os.path.dirname(__file__)
)  # append current directory so relative imports can work
sys.path.append("/tmp/") # NOSONAR
import ast
import difflib
import importlib
import inspect
import json
import logging
import math
import re
import subprocess
import traceback
import unittest
from threading import Thread

import numpy as np
from constants import (
    ALLOWLIST_IMPORTS,
    FORBID_ACCESS,
    FORBID_STRINGS,
    REWARD_FUNCTION_PATH,
    TIMEOUT_SEC,
    TRACK_NAME_PATH,
)

REWARD_FUNCTION_FILE = "/tmp/reward_function.py" # NOSONAR
TRACK_PARSE_ERROR = "InternalServerError: Unable to parse track information"

logger = logging.getLogger()
logger.setLevel(logging.INFO)


class DeepRacerError(Exception):
    def __init__(self, **kwargs):
        self._dict = kwargs

    def __str__(self):
        return json.dumps(self._dict)


def wrap(fn):
    def f(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except DeepRacerError:
            raise
        except Exception:
            _, exc_value, exc_traceback = sys.exc_info()
            tb = traceback.extract_tb(exc_traceback)[-1]
            exc = traceback.format_exc().splitlines()[-1]
            is_syntax_error = isinstance(exc_value, SyntaxError)
            filename = exc_value.filename if is_syntax_error else tb[0]
            lineno = exc_value.lineno if is_syntax_error else tb[1]
            line = exc_value.text if is_syntax_error else tb[3]
            _, message = exc.split(":", 1)

            if is_syntax_error:
                error_type = "SYNTAX_ERROR"
            elif isinstance(exc_value, ImportError):
                error_type = "IMPORT_ERROR"
            else:
                error_type = "TEST_FAILURE"

            _dict = {
                "type": error_type,
                "message": message.strip(),
                "line": line.strip(),
            }
            if filename.endswith("reward_function.py"):
                _dict["lineNumber"] = lineno

            raise DeepRacerError(**_dict)

    return f


def _parse_flake8_line(line, reward_content):
    """Parse a single flake8 output line into a structured result."""
    parts = line.split(":")

    if len(parts) > 3:
        error_type = parts[2].strip()
        line_num = parts[1].strip()
        message = " : ".join(parts[3:])  # Join remaining parts as message
    elif len(parts) == 3:
        error_type = parts[0].strip()
        line_num = parts[1].strip()
        message = parts[2]
    else:
        error_type = "Unexpected Error"
        line_num = "1"
        message = line.strip()

    return {
        "type": error_type,
        "message": message,
        "line": (
            reward_content[int(line_num) - 1].strip()
            if int(line_num) <= len(reward_content)
            else ""
        ),
        "lineNumber": line_num,
    }


def run_flake8():
    try:
        result = subprocess.run(
            [
                "flake8",
                REWARD_FUNCTION_PATH,
                "--format=%(code)s:%(row)d:%(text)s",
                "--select=E,F",
                "--ignore=E5,E226,E261",  # ignore stylistic errors
            ],
            capture_output=True,
            text=True,
        )
        if result.stdout:
            lines = result.stdout.strip().split("\n")

            parsed_results = []
            try:
                with open(REWARD_FUNCTION_PATH) as reward_file:
                    reward_content = reward_file.readlines()

                for line in lines:
                    if line.strip():
                        parsed_results.append(_parse_flake8_line(line, reward_content))

            except (ValueError, IndexError, FileNotFoundError) as e:
                # If parsing fails, return generic error
                return [
                    {"type": "Unexpected Error", "message": str(e), "lineNumber": "1"}
                ]

            return parsed_results
        return []
    except Exception as e:
        raise DeepRacerError(f"Flake8 linting error: {e}")


def _fail_import(*args, **kwargs):
    msg = "Import should be put at the top of file, module: " + args[0]
    raise DeepRacerError(message=msg, type="IMPORT_ERROR")


class TestSyntax(unittest.TestCase):
    def test_syntax(self):
        lint_output = run_flake8()
        for result in lint_output:
            raise DeepRacerError(**result)


# The recursive pattern below traverses the main AST (parse) and
# utilizes a helper function (_parse_chain) to walk any attributes or calls for names present
# ref: https://stackoverflow.com/questions/72064609/how-can-i-retrieve-function-names-and-attributes-from-python-code-with-ast
def _parse_chain(d, c, p=[]):
    """Walk attributes or calls to extract the full dotted name."""
    if isinstance(d, ast.Name):
        return [d.id] + p
    if isinstance(d, ast.Call):
        for i in d.args:
            parse(i, c)
        return _parse_chain(d.func, c, p)
    if isinstance(d, ast.Attribute):
        return _parse_chain(d.value, c, [d.attr] + p)


def parse(d, c):
    """Traverse AST and collect function/attribute names into list c."""
    if isinstance(d, (ast.Call, ast.Attribute)):
        c.append(".".join(_parse_chain(d, c)))
    else:
        for i in getattr(d, "_fields", []):
            t = getattr(d, i)
            if isinstance(t, list):
                for i in t:
                    parse(i, c)
            else:
                parse(t, c)


class TestIllegalImportsAndBuiltins(unittest.TestCase):
    @wrap
    def test_imports(self):
        with open(REWARD_FUNCTION_FILE) as source:
            tree = ast.parse(source.read())
        analyzer = Analyzer()
        analyzer.visit(tree)
        all_imports = analyzer.report()
        # only care about the upper most module
        all_imports = [x.split(".")[0] for x in all_imports]
        if any(x not in ALLOWLIST_IMPORTS for x in all_imports):
            fail(
                "The reward function contains illegal import(s): {}".format(
                    [x for x in all_imports if x not in ALLOWLIST_IMPORTS]
                )
            )

    @wrap
    def test_builtins(self):
        with open(REWARD_FUNCTION_FILE) as source:
            tree = ast.parse(source.read())
        results = []
        parse(tree, results)
        for method in results:
            if method in FORBID_ACCESS:
                fail(
                    "The reward function contains forbidden builtins: {}".format(
                        [x for x in results if x in FORBID_ACCESS]
                    )
                )

    @wrap
    def test_forbidden_strings(self):
        with open(REWARD_FUNCTION_FILE) as source:
            tree = source.read()
            for forbiddenString in FORBID_STRINGS:
                if forbiddenString in tree:
                    fail(
                        'Your code snippet contains a restricted string "{}" that is not allowed. Please remove or replace the string and try again.'.format(
                            forbiddenString
                        )
                    )


_fa = {}
_fa_counter = {}


def get_function_name(forbidden):
    def temp_func(*args, **kwargs):
        global _fa_counter
        global _fa
        _fa_counter[forbidden] += 1
        return _fa[forbidden](*args, **kwargs)

    return temp_func


class TestUnsafeBuiltins(unittest.TestCase):
    @wrap
    def test_unsafe_builtins(self):
        # read track name
        with open(TRACK_NAME_PATH) as f:
            track_name = f.readlines()
        # get waypoints
        waypoints_path = os.path.join(f"routes/{track_name[0]}.npy")
        waypoints = list([])
        try:
            waypoints = np.load(waypoints_path)
        except Exception:
            raise DeepRacerError(TRACK_PARSE_ERROR)
        valid_params, _, _, _, _ = get_params(waypoints)

        global _fa_counter
        global _fa
        for forbidden, _ in FORBID_ACCESS.items():
            _fa[forbidden] = __builtins__[forbidden]
            _fa_counter[forbidden] = 0
            __builtins__[forbidden] = get_function_name(forbidden)
        import reward_function

        # need to reload the reward_function for new requests
        importlib.reload(reward_function)
        rf = reward_function.reward_function
        rf(valid_params)
        for unforbidden, val in _fa.items():
            __builtins__[unforbidden] = val
        unsafe_builtins = []
        for forbidden, count in FORBID_ACCESS.items():
            if _fa_counter[forbidden] > count:
                unsafe_builtins.append(f"{forbidden}, times: {_fa_counter[forbidden]}")
        if len(unsafe_builtins) > 0:
            fail(f'Unsafe builtin function detected: "{unsafe_builtins}".')


class TestImportsWithinModule(unittest.TestCase):
    @wrap
    def test_imports_within_module(self):
        # Import reward function
        import reward_function

        # need to reload the reward_function for new requests
        importlib.reload(reward_function)

        # read track name
        with open(TRACK_NAME_PATH) as f:
            track_name = f.readlines()
        # get waypoints
        waypoints_path = os.path.join(f"routes/{track_name[0]}.npy")
        waypoints = list([])
        try:
            waypoints = np.load(waypoints_path)
        except Exception:
            raise DeepRacerError(TRACK_PARSE_ERROR)
        rf = reward_function.reward_function
        valid_params, _, _, _, _ = get_params(waypoints)
        _imp = __builtins__["__import__"]
        __builtins__["__import__"] = _fail_import
        try:
            rf(valid_params)
        except Exception as e:
            fail(f'Unsafe builtin function detected: "{e}".')
        finally:
            __builtins__["__import__"] = _imp


def get_params(waypoints):
    # get track width
    track_width = math.sqrt(
        (waypoints[0, 4] - waypoints[0, 2]) ** 2
        + (waypoints[0, 5] - waypoints[0, 3]) ** 2
    )
    num_waypoints = waypoints.shape[0]

    # calculate track length
    track_length = 0
    for i in range(1, waypoints.shape[0]):
        track_length += math.sqrt(
            (waypoints[i, 2] - waypoints[i - 1, 2]) ** 2
            + (waypoints[i, 3] - waypoints[i - 1, 3]) ** 2
        )

    # set a valid input as a startline
    valid_params = {
        "all_wheels_on_track": True,
        "x": (waypoints[0, 2] + waypoints[0, 0])
        / 2,  # start line between inner lane and center lane
        "y": (waypoints[0, 3] + waypoints[0, 1])
        / 2,  # start line between inner lane and center lane
        "distance_from_center": 0,
        "heading": 0,
        "progress": 0,
        "steps": 1,
        "speed": 0.5,
        "steering_angle": 6,
        "track_width": track_width,
        "waypoints": waypoints[:, 2:4],  # only the center line
        "closest_waypoints": [0, 1],
        "is_left_of_center": True,
        "is_reversed": True,
        "track_length": track_length,
        "closest_objects": [0, 1],  # random, doesn't matter
        "objects_location": [
            [4.511289152034186, 1.3292364463761641],
            [6.537302737755836, 1.4140104486149618],
            [4.752976532490525, 3.1350845729056838],
            [3.103370840828792, 4.133062703357412],
            [0.7094212601659824, 4.217507179944688],
            [1.5996645329306798, 1.7124666440529925],
        ],
        "objects_left_of_center": [True, True, False, True, False, True],
        "object_in_camera": True,
        "objects_speed": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        "objects_heading": [
            2.717322296283114,
            2.368920328229894,
            -1.9305073380382327,
            -1.3586571052564007,
            0.00025041038280975884,
            0.5573269195381345,
        ],
        "objects_distance": [
            1.9501724549506574,
            4.139700164331426,
            7.997164727523002,
            10.024219705986598,
            12.56561517097048,
            15.090712948383509,
        ],
        "is_crashed": False,
        "is_offtrack": False,
    }

    start_car = {
        "all_wheels_on_track": True,
        "x": waypoints[0, 2],  # start line
        "y": waypoints[0, 3],  # start line
        "distance_from_center": 0,
        "heading": 0,
        "progress": 0,
        "steps": 1,
        "speed": 0.1,
        "steering_angle": 0.2,
        "track_width": track_width,
        "waypoints": waypoints[:, 2:4],  # only the center line
        "closest_waypoints": [0, 1],  # doesn't matter
        "is_left_of_center": True,
        "is_reversed": True,
        "track_length": track_length,
        "closest_objects": [0, 1],
        "objects_location": [
            [4.511289152034186, 1.3292364463761641],
            [6.537302737755836, 1.4140104486149618],
            [4.752976532490525, 3.1350845729056838],
            [3.103370840828792, 4.133062703357412],
            [0.7094212601659824, 4.217507179944688],
            [1.5996645329306798, 1.7124666440529925],
        ],
        "objects_left_of_center": [True, True, False, True, False, True],
        "object_in_camera": True,
        "objects_speed": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        "objects_heading": [
            2.717322296283114,
            2.368920328229894,
            -1.9305073380382327,
            -1.3586571052564007,
            0.00025041038280975884,
            0.5573269195381345,
        ],
        "objects_distance": [
            1.9501724549506574,
            4.139700164331426,
            7.997164727523002,
            10.024219705986598,
            12.56561517097048,
            15.090712948383509,
        ],
        "is_crashed": False,
        "is_offtrack": False,
    }

    progress_car = {
        "all_wheels_on_track": True,
        "x": waypoints[math.floor(num_waypoints / 2), 2],  # middle waypoint
        "y": waypoints[math.floor(num_waypoints / 2), 3],  # middle waypoint
        "distance_from_center": 0,
        "heading": 359.9,
        "progress": 50,
        "steps": 100,
        "speed": 1.0,
        "steering_angle": 6,
        "track_width": track_width,
        "waypoints": waypoints[:, 2:4],  # only the center line
        "closest_waypoints": [3, 4],
        "is_left_of_center": True,
        "is_reversed": True,
        "track_length": track_length,
        "closest_objects": [0, 1],
        "objects_location": [
            [4.511289152034186, 1.3292364463761641],
            [6.537302737755836, 1.4140104486149618],
            [4.752976532490525, 3.1350845729056838],
            [3.103370840828792, 4.133062703357412],
            [0.7094212601659824, 4.217507179944688],
            [1.5996645329306798, 1.7124666440529925],
        ],
        "objects_left_of_center": [True, True, False, True, False, True],
        "object_in_camera": True,
        "objects_speed": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        "objects_heading": [
            2.717322296283114,
            2.368920328229894,
            -1.9305073380382327,
            -1.3586571052564007,
            0.00025041038280975884,
            0.5573269195381345,
        ],
        "objects_distance": [
            1.9501724549506574,
            4.139700164331426,
            7.997164727523002,
            10.024219705986598,
            12.56561517097048,
            15.090712948383509,
        ],
        "is_crashed": False,
        "is_offtrack": False,
    }
    # get offtrack coordinates by drawing the line from center to outer lane and making it double the track width.
    dist = math.sqrt(
        (waypoints[0, 4] - waypoints[0, 0]) ** 2
        + (waypoints[0, 5] - waypoints[0, 1]) ** 2
    )
    slope_x = (waypoints[0, 4] - waypoints[0, 0]) / dist
    slope_y = (waypoints[0, 5] - waypoints[0, 1]) / dist
    offtrack_x = (
        waypoints[0, 0] + slope_x * track_width
    )  # distance from center is equal to track width
    offtrack_y = (
        waypoints[0, 1] + slope_y * track_width
    )  # distance from center is equal to track width
    off_track_car = {
        "all_wheels_on_track": False,
        "x": offtrack_x,
        "y": offtrack_y,
        "distance_from_center": track_width,
        "heading": 359.9,
        "progress": 0,
        "steps": 1,
        "speed": 1,
        "steering_angle": 15,
        "track_width": track_width,
        "waypoints": waypoints[:, 2:4],  # only the center line
        "closest_waypoints": [0, 1],
        "is_left_of_center": True,
        "is_reversed": True,
        "track_length": track_length,
        "closest_objects": [0, 1],
        "objects_location": [
            [4.511289152034186, 1.3292364463761641],
            [6.537302737755836, 1.4140104486149618],
            [4.752976532490525, 3.1350845729056838],
            [3.103370840828792, 4.133062703357412],
            [0.7094212601659824, 4.217507179944688],
            [1.5996645329306798, 1.7124666440529925],
        ],
        "objects_left_of_center": [True, True, False, True, False, True],
        "object_in_camera": True,
        "objects_speed": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        "objects_heading": [
            2.717322296283114,
            2.368920328229894,
            -1.9305073380382327,
            -1.3586571052564007,
            0.00025041038280975884,
            0.5573269195381345,
        ],
        "objects_distance": [
            1.9501724549506574,
            4.139700164331426,
            7.997164727523002,
            10.024219705986598,
            12.56561517097048,
            15.090712948383509,
        ],
        "is_crashed": False,
        "is_offtrack": True,
    }

    finish_car = {
        "all_wheels_on_track": True,
        "x": waypoints[num_waypoints - 1, 2],  # finish line
        "y": waypoints[num_waypoints - 1, 3],  # finish line
        "distance_from_center": 0,
        "heading": 359.9,
        "progress": 100,
        "steps": 10000,
        "speed": 5.0,
        "steering_angle": 0,
        "track_width": track_width,
        "waypoints": waypoints[:, 2:4],  # only the center line
        "closest_waypoints": [len(waypoints) - 2, len(waypoints) - 1],
        "is_left_of_center": True,
        "is_reversed": True,
        "track_length": track_length,
        "closest_objects": [0, 1],
        "objects_location": [
            [4.511289152034186, 1.3292364463761641],
            [6.537302737755836, 1.4140104486149618],
            [4.752976532490525, 3.1350845729056838],
            [3.103370840828792, 4.133062703357412],
            [0.7094212601659824, 4.217507179944688],
            [1.5996645329306798, 1.7124666440529925],
        ],
        "objects_left_of_center": [True, True, False, True, False, True],
        "object_in_camera": True,
        "objects_speed": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
        "objects_heading": [
            2.717322296283114,
            2.368920328229894,
            -1.9305073380382327,
            -1.3586571052564007,
            0.00025041038280975884,
            0.5573269195381345,
        ],
        "objects_distance": [
            1.9501724549506574,
            4.139700164331426,
            7.997164727523002,
            10.024219705986598,
            12.56561517097048,
            15.090712948383509,
        ],
        "is_crashed": False,
        "is_offtrack": False,
    }
    return valid_params, start_car, progress_car, off_track_car, finish_car


class TestRewardFunction(unittest.TestCase):
    @wrap
    def setUp(self):
        import reward_function

        # need to reload the reward_function for new requests
        importlib.reload(reward_function)

        # read track name
        with open(TRACK_NAME_PATH) as f:
            self.track_name = f.readlines()
        # get waypoints
        waypoints_path = os.path.join(f"routes/{self.track_name[0]}.npy")
        self.waypoints = list([])
        try:
            self.waypoints = np.load(waypoints_path)
        except Exception:
            raise DeepRacerError("InternalServerError: Unable to parse track information")
        self.rf = reward_function.reward_function
        (
            self.valid_params,
            self.start_car,
            self.progress_car,
            self.off_track_car,
            self.finish_car,
        ) = get_params(self.waypoints)

    @wrap
    def test_reward_function_signature(self):
        parameters = inspect.signature(self.rf).parameters
        if len(parameters.keys()) != 1:
            fail(
                "Invalid reward function signature. Should be def reward_function(params)"
            )

    @wrap
    def test_all_imported_modules(self):
        self.rf(self.valid_params)

    @wrap
    def test_return_type(self):
        reward = self.rf(self.valid_params)
        if not isinstance(reward, float):
            fail(f"Method returned non-floating type value: {reward}")

    @wrap
    def test_reward_range(self):
        reward = self.rf(self.valid_params)
        if not self._reward_in_range(reward):
            fail(
                "Reward score out of range. Reward: {}, range: {}".format(
                    reward, "[-1e5, 1e5]"
                )
            )

    @staticmethod
    def _reward_in_range(reward):
        return isinstance(reward, float) and -1e5 <= reward <= 1e5

    @wrap
    def test_start_car(self):
        reward = self.rf(self.start_car)
        if not self._reward_in_range(reward):
            fail(f"Vehicle failed at start position. Reward: {reward}")

    @wrap
    def test_progress_car(self):
        reward = self.rf(self.progress_car)
        if not self._reward_in_range(reward):
            fail(f"Vehicle failed to make progress. Reward: {reward}")

    @wrap
    def test_off_track_car(self):
        reward = self.rf(self.off_track_car)
        if not self._reward_in_range(reward):
            fail(f"Off-track vehicle failed to make progress. Reward: {reward}")

    @wrap
    def test_finish_car(self):
        reward = self.rf(self.finish_car)
        if not self._reward_in_range(reward):
            fail(f"Vehicle failed to make it to end of track. Reward: {reward}")


def fail(msg=None):
    raise DeepRacerError(message=msg, type="TEST_FAILURE")


class Analyzer(ast.NodeVisitor):
    def __init__(self):
        self.stats = set()

    def visit_Import(self, node):
        for alias in node.names:
            # self.stats["import"].append(alias.name)
            self.stats.add(alias.name)
        self.generic_visit(node)

    def visit_ImportFrom(self, node):
        for alias in node.names:
            self.stats.add(node.module)
        self.generic_visit(node)

    def report(self):
        return self.stats


def build_syntax_and_import_suite():
    suite = unittest.TestSuite()
    suite.addTest(TestSyntax("test_syntax"))
    suite.addTest(TestIllegalImportsAndBuiltins("test_imports"))
    suite.addTest(TestIllegalImportsAndBuiltins("test_builtins"))
    suite.addTest(TestIllegalImportsAndBuiltins("test_forbidden_strings"))
    return suite


def build_unsafe_builtins_suite():
    suite = unittest.TestSuite()
    # This dynamic test raises false flags for builtins, is additional to the static tests
    # which cover all scenarios. TODO: Investigate more of any better test cases to be added.
    # suite.addTest(TestUnsafeBuiltins('test_unsafe_builtins'))
    suite.addTest(TestImportsWithinModule("test_imports_within_module"))
    return suite


def build_runtime_suite():
    suite = []
    suite.append("test_reward_function_signature")
    suite.append("test_all_imported_modules")
    suite.append("test_return_type")
    suite.append("test_reward_range")
    suite.append("test_start_car")
    suite.append("test_progress_car")
    suite.append("test_off_track_car")
    suite.append("test_finish_car")
    return suite


multiThreadTestResults = {}


def run_suites(reward_function, track_name):
    with open(REWARD_FUNCTION_PATH, "w") as f:
        f.write(reward_function)
    # save track name for reference by validator
    with open(TRACK_NAME_PATH, "w") as f:
        f.write(track_name)

    logger.info("Reward function passed in arg: " + reward_function)
    # Verify saved rf is same as rf passed in arg
    with open(REWARD_FUNCTION_PATH) as f:
        saved_rf = f.read()
        output_list = [
            li for li in difflib.ndiff(reward_function, saved_rf) if li[0] != " "
        ]
        logger.info("Reward function diff: " + ",".join(output_list))

    suites = [build_syntax_and_import_suite(), build_unsafe_builtins_suite()]
    for suite in suites:
        test_results = unittest.TextTestRunner(failfast=True).run(suite)
        if test_results.errors or test_results.failures:
            return process_results(test_results)
    # Run the following tests in parallel
    try:
        threads = []
        for suite in build_runtime_suite():
            threads.append(Thread(target=threaded_test, args=(suite,)))
        # Run the threads
        for t in threads:
            # Does not block main thread from exiting
            t.setDaemon(True)
            t.start()
        # Wait for threads to finish
        for t in threads:
            # Only observe till timeout threshold
            t.join(TIMEOUT_SEC)
            # If thread is still alive throw timeout exception
            if t.is_alive():
                test_results = ThreadedTestException(errMsg=[["", "Timed Out"]])
                break
        global multiThreadTestResults
        # Evaluate threaded tests results
        for _, v in multiThreadTestResults.items():
            if v.errors or v.failures:
                test_results = v
                break
    except Exception as e:
        raise DeepRacerError(f"InternalServerError: {str(e)}")
    return process_results(test_results)


class ThreadedTestException:
    def __init__(self, errMsg=""):
        self.errors = errMsg
        self.failures = []


def threaded_test(testName):
    global multiThreadTestResults
    test_results = unittest.TextTestRunner(failfast=True).run(
        TestRewardFunction(testName)
    )
    multiThreadTestResults[testName] = test_results


def process_results(test_results):
    return list(
        map(
            lambda e: extract_failure_message(e[1]),
            test_results.errors + test_results.failures,
        )
    )


def extract_failure_message(txt):
    line = txt.splitlines()[-1]
    try:
        return json.loads(re.search("DeepRacerError: (.+)", line).group(1))
    except Exception:
        pass
    return json.loads(str(DeepRacerError(message=line, type="TEST_FAILURE")))
