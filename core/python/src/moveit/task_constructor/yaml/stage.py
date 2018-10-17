#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains support for saving a stage's current properties
to YAML and loading them later.
"""

__all__ = ['toyaml', 'fromyaml']
__author__ = 'Jan Ebert'

import yaml

from moveit.task_constructor import stages
from moveit.task_constructor import core
from moveit.task_constructor.yaml import rosmsg
from moveit.task_constructor.yaml import utils


def _represent_stage(dumper, stage):
    """Return a stage in YAML format.

    Used as a PyYAML `multi_representer` for the base class
    `moveit.task_constructor.core.Stage`.
    """
    return dumper.represent_mapping(type(stage).__name__,
            {'stage':
                {'name': stage.name,
                'properties': dict(stage.properties)}
            })


def _set_property(stage, prop):
    """Set the given stage's property corresponding to the name and
    value contained in the given property.
    """
    name_node, value_node = prop
    name = rosmsg._node_to_value(name_node)
    value = rosmsg._node_to_value(value_node)
    if value is not None:
        setattr(stage, name, value)


def _set_properties(stage, properties):
    """Set the given stage's properties corresponding to the names and
    values contained in the given properties.

    The given properties are a PyYAML node.
    """
    map(lambda prop: _set_property(stage, prop), properties)


def _construct_stage(node):
    """Construct a stage from the given PyYAML node."""
    stage_class_str = node.tag
    try:
        stage_class = getattr(stages, stage_class_str)
    except AttributeError as ex:
        try:
            stage_class = getattr(core, stage_class_str)
        except AttributeError as ex2:
            utils.raisefrom(ex2, utils.TypeNotFoundError(stage_class_str))
    stage = stage_class()

    value = node.value[0][1].value  # values are nested in a dictionary
    stage.name = str(value[0][1].value)
    # set stage properties
    properties = value[1][1].value
    _set_properties(stage, properties)
    return stage


def toyaml(stage):
    return yaml.dump(stage)


def fromyaml(yml_str):
    node = utils.node_from_string(yml_str)
    return _construct_stage(node)


yaml.add_multi_representer(core.Stage, _represent_stage)

