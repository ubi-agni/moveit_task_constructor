"""This module initializes the MTC YAML package and provides minor
convenience functions.
"""

from yaml import dump, load

import rosmsg
import rostime
import stage
import utils


def toYaml(obj):
    return dump(obj, Dumper=utils.Dumper)


def toFile(obj, path):
    with open(path, 'w') as f:
        dump(obj, f, Dumper=utils.Dumper)


def fromYaml(yml_str):
    return load(yml_str, Loader=utils.Loader)

