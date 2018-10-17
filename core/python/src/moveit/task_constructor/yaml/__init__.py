"""This module initializes the MTC YAML package and provides minor
convenience functions.
"""

from yaml import dump

import rosmsg
import rostime
import stage
import utils


def toyaml(obj):
    return dump(obj)


def tofile(obj, path):
    with open(path, 'w') as f:
        dump(obj, f)

