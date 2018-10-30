#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a ROS message's current
properties to YAML and loading them later.
"""

from __future__ import unicode_literals

__all__ = ['toyaml', 'fromyaml']
__author__ = 'Jan Ebert'

import __builtin__

from genpy import Message
import roslib.message
from yaml import add_multi_representer, add_multi_constructor

from moveit.task_constructor.yaml import rostime
from moveit.task_constructor.yaml import utils


TAG_PREFIX = 'ROSMsg/'
"""How to prefix ROS messages for the multi constructor.

Arbitrary; but changing this value will break compatibility.
"""


def _represent_msg(dumper, msg):
    """Return a ROS message in YAML format.

    Used as a PyYAML `multi_representer` for the base class
    `genpy.Message`.
    """
    return dumper.represent_mapping(TAG_PREFIX + msg._type,
            {name: getattr(msg, name) for name in msg.__slots__})


def _construct_msg(loader, tag_suffix, node):
    """Construct a ROS message from the given PyYAML node."""
    # from rostopic.create_publisher()
    # tag_suffix like: 'moveit_task_constructor_msgs/Property'

    # get msg class and construct
    try:
        msg_class = roslib.message.get_message_class(tag_suffix)
    except ValueError as ex:
        utils.raisefrom(ex, utils.TypeNotFoundError(tag_suffix))
    if msg_class is None:
        raise utils.TypeNotFoundError(tag_suffix)
    msg = msg_class()

    # set msg properties
    properties = loader.construct_mapping(node)
    utils.attributes_from_dict(msg, properties)
    return msg


add_multi_representer(Message, _represent_msg, Dumper=utils.Dumper)
add_multi_constructor(TAG_PREFIX, _construct_msg, Loader=utils.Loader)

