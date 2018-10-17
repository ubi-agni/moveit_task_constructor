#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a ROS time's current
properties to YAML and loading them later.
"""

from __future__ import print_function

__all__ = ['toyaml', 'fromyaml']
__author__ = 'Jan Ebert'

from genpy import rostime
import yaml

from moveit.task_constructor.yaml import utils


def _represent_rostime(dumper, rostime):
    return dumper.represent_mapping(utils.formattypeof(rostime),
            {'secs': rostime.secs, 'nsecs': rostime.nsecs})


def _construct_rostime(node):
    cls_start_ix = node.tag.rfind('.') + 1
    cls_str = node.tag[cls_start_ix:]
    cls = getattr(rostime, cls_str)
    args = {namenode.value: int(valnode.value)
            for namenode, valnode in node.value}
    return cls(**args)


def toyaml(rostime):
    return yaml.dump(rostime)


def fromyaml(node):
    return _construct_rostime(node)


yaml.add_multi_representer(rostime.TVal, _represent_rostime)

