"""This module initializes the MTC YAML package and provides minor
convenience functions.
"""

from __future__ import absolute_import, print_function

try:
    from yaml import CLoader as BaseLoader, CDumper as Dumper
except ImportError:
    from yaml import Loader as BaseLoader, Dumper
from yaml import dump, load
from yaml.constructor import *
from yaml.nodes import ScalarNode, SequenceNode, MappingNode

import types, importlib
import genpy, roslib.message
import moveit.task_constructor.core as core


def full_type_name(cls):
    if not isinstance(cls, type):
        cls = type(cls)
    return cls.__module__ + '.' + cls.__name__


class Loader(BaseLoader):
    def add_class_constructor(self, cls, constructor):
        self.add_constructor(full_type_name(cls), lambda loader, node: constructor(loader, cls, node))
    add_class_constructor = classmethod(add_class_constructor)

    def _construct_ros_msg(self, type_tag, node):
        """Construct a ROS message from the given PyYAML node."""
        try:
            cls = roslib.message.get_message_class(type_tag)
        except ValueError as ex:
            return None

        args = self.construct_mapping(node)
        return cls(**args)

    def _construct_stage(loader, type_tag, node):
        """Construct a stage from the given PyYAML node."""
        try:
            package, type_str = type_tag.rsplit('.', 1)
            module = importlib.import_module(package)
            cls = getattr(module, type_str)
        except (ImportError, AttributeError):
            return None

        stage = cls()
        attributes = loader.construct_mapping(node)
        stage.name = attributes['name']
        stage.properties.update(attributes['properties'])
        return stage

    def construct_object(self, node, deep=False):
        if node in self.constructed_objects:
            return self.constructed_objects[node]
        if deep:
            old_deep = self.deep_construct
            self.deep_construct = True
        if node in self.recursive_objects:
            raise ConstructorError(None, None,
                    "found unconstructable recursive node", node.start_mark)
        self.recursive_objects[node] = None
        constructor = None
        if node.tag in self.yaml_constructors:
            constructor = self.yaml_constructors[node.tag]
            data = constructor(self, node)
        else:
            for constructor in [self._construct_ros_msg, self._construct_stage]:
                data = constructor(node.tag, node)
                if data is not None:
                    break
            else:
                if isinstance(node, ScalarNode):
                    data = self.__class__.construct_scalar(self, node)
                elif isinstance(node, SequenceNode):
                    data = self.__class__.construct_sequence(self, node)
                elif isinstance(node, MappingNode):
                    data = self.__class__.construct_mapping(self, node)
                else:
                    raise ConstructorError(None, None, "Unknown type: " + node.tag, node.start_mark)

        if isinstance(data, types.GeneratorType):
            generator = data
            data = generator.next()
            if self.deep_construct:
                for dummy in generator:
                    pass
            else:
                self.state_generators.append(generator)

        self.constructed_objects[node] = data
        del self.recursive_objects[node]
        if deep:
            self.deep_construct = old_deep
        return data


# Handling of ros Time + Duration
def _represent_rostime(dumper, value):
    """Dump ROS Time or Duration to YAML."""
    return dumper.represent_mapping(full_type_name(value), {'secs': value.secs, 'nsecs': value.nsecs})


def _construct_rostime(loader, cls, node):
    args = loader.construct_mapping(node)
    return cls(**args)


Dumper.add_multi_representer(genpy.TVal, _represent_rostime)
Loader.add_class_constructor(genpy.Time, _construct_rostime)
Loader.add_class_constructor(genpy.Duration, _construct_rostime)


# Handling of ROS messages
def _represent_ros_msg(dumper, msg):
    """Dump a ROS message to YAML."""
    return dumper.represent_mapping(msg._type, {name: getattr(msg, name) for name in msg.__slots__})

Dumper.add_multi_representer(genpy.Message, _represent_ros_msg)


# Handling of Stages
def _represent_stage(dumper, stage):
    """Dump a MTC stage to YAML."""
    values = []
    node = MappingNode(full_type_name(stage), values, flow_style=False)
    try:
        name = stage.name
    except AttributeError:
        name = "task"
    values.append((dumper.represent_data("name"), dumper.represent_data(name)))
    values.append((dumper.represent_data("properties"), dumper.represent_mapping(None, stage.properties)))
    if isinstance(stage, core.ContainerBase) or isinstance(stage, core.Task):
        children_list = []
        children_node = SequenceNode(None, children_list)
        values.append((dumper.represent_data("children"), children_node))
    return node

Dumper.add_multi_representer(core.Stage, _represent_stage)
Dumper.add_multi_representer(core.Task, _represent_stage)


def toYaml(obj):
    return dump(obj, Dumper=Dumper)


def toFile(obj, path):
    with open(path, 'w') as f:
        dump(obj, f, Dumper=Dumper)


def fromYaml(yml_str):
    return load(yml_str, Loader=Loader)
