#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a ROS time's current
properties to YAML and loading them later.
"""

from __future__ import print_function, unicode_literals

__all__ = ['toyaml', 'fromyaml']
__author__ = 'Jan Ebert'

from genpy import rostime
from yaml import add_multi_representer, add_multi_constructor

from moveit.task_constructor.yaml import utils


TAG_PREFIX = 'ROSTime.'
"""How to prefix ROS times for the multi constructor.

Arbitrary; but changing this value will break compatibility.
"""


def _represent_rostime(dumper, rostime_):
    return dumper.represent_mapping(TAG_PREFIX + type(rostime_).__name__,
            {'secs': rostime_.secs, 'nsecs': rostime_.nsecs})


def _construct_rostime(loader, tag_suffix, node):
    cls = getattr(rostime, tag_suffix)
    args = loader.construct_mapping(node)
    return cls(**args)


add_multi_representer(rostime.TVal, _represent_rostime, Dumper=utils.Dumper)
add_multi_constructor(TAG_PREFIX, _construct_rostime,
        Loader=utils.Loader)

