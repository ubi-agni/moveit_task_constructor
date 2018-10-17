#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a ROS message's current
properties to YAML and loading them later.
"""

__all__ = ['toyaml', 'fromyaml']
__author__ = 'Jan Ebert'

import __builtin__

from genpy import Message
import roslib.message
import yaml

from moveit.task_constructor.yaml import rostime
from moveit.task_constructor.yaml import utils


def _represent_msg(dumper, msg):
    """Return a ROS message in YAML format.

    Used as a PyYAML `multi_representer` for the base class
    `genpy.Message`.
    """
    return dumper.represent_mapping(msg._type,
            {name: getattr(msg, name) for name in msg.__slots__})


def _node_to_value(node):
    """Return the value contained in the PyYAML node cast to the type
    specified by the node's tag.
    """
    tag, value = node.tag, node.value
    if tag.startswith('tag:'):
        tag_start = tag.rfind(':') + 1
        tag = tag[tag_start:]
        if tag == 'seq':  # lists are saved with the 'seq' tag
            cls = list
        elif tag == 'map':  # dicts are saved with the 'map' tag
            cls = dict
        elif tag == 'bool':  # convert string boolean values
            return value.lower() != 'false'
        elif tag == 'null':
            return None
        elif tag.endswith('MergeMode'):
            return value[0].value
        else:
            try:
                cls = getattr(__builtin__, tag)
            except AttributeError as ex:
                utils.raisefrom(ex, utils.TypeNotFoundError(tag))
        return cls(value)
    elif tag.startswith('genpy.rostime.'):
        return rostime._construct_rostime(node)
    else:
        return _construct_msg(node)


def _attribute_from_property(obj, prop):
    """Set the given object's attribute corresponding to the name and
    value contained in the given property.
    """
    name_node, value_node = prop
    setattr(obj, _node_to_value(name_node), _node_to_value(value_node))


def _attributes_from_properties(obj, properties):
    """Set the given object's attributes corresponding to the names and
    values contained in the given properties.
    """
    map(lambda prop: _attribute_from_property(obj, prop), properties)


def _construct_msg(node):
    """Construct a ROS message from the given PyYAML node."""
    # from rostopic.create_publisher()
    # msg_class_str like: 'moveit_task_constructor_msgs/Property'

    # get msg class and construct
    msg_class_str = node.tag
    try:
        msg_class = roslib.message.get_message_class(msg_class_str)
    except ValueError as ex:
        utils.raisefrom(ex, utils.TypeNotFoundError(msg_class_str))
    if msg_class is None:
        raise utils.TypeNotFoundError(msg_class_str)
    msg = msg_class()

    # set msg properties
    properties = node.value
    _attributes_from_properties(msg, properties)
    return msg


def toyaml(rosmsg):
    return yaml.dump(rosmsg)


def fromyaml(yml_str):
    node = utils.node_from_string(yml_str)
    return _construct_msg(node)


yaml.add_multi_representer(Message, _represent_msg)

