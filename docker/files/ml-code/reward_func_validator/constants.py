# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

REWARD_FUNCTION_PATH = "/tmp/reward_function.py" # NOSONAR
TRACK_NAME_PATH = "/tmp/track_name" # NOSONAR

ALLOWLIST_IMPORTS = ["math", "random", "numpy", "scipy", "shapely"]
# compile and exec are not set to 0 because
# import reward function makes compile and exec trigger atleast once
FORBID_ACCESS = {"open": 0, "exec": 2, "eval": 0, "compile": 1, "input": 0}

FORBID_STRINGS = {
    "__import__": 0,
    "load": 0,
    "fromfile": 0,
    "genfromtxt": 0,
    "loadtxt": 0,
    "tofile": 0,
    "savez": 0,
    "save": 0,
    "savetxt": 0,
    "memmap": 0,
}

TIMEOUT_SEC = 5
