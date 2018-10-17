#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains utility functions and classes for converting to
YAML such as exceptions, Python version checking and type formatting.
"""

__author__ = 'Jan Ebert'

import sys
import traceback

import yaml


class TypeNotFoundError(Exception):
    """Signal that the given type was not found in the supported
    modules.
    """

    def __init__(self, type_str, message=None):
        """Construct the exception with the given string of the type not
        supported and an optional custom message.
        """
        if message is None:
            self.message = "Type '{}' was not found in the supported " \
                    "modules.".format(type_str)
        else:
            self.message = message

    def __str__(self):
        return self.message


def raisefrom(from_ex, raise_ex):
    """Raise the second given exception from the traceback of the first
    given exception.

    Mimics the Python3 `raise ... from ...` construct (but with the
    arguments swapped).
    """
    if pyge3:
        raise raise_ex.with_traceback(from_ex.__traceback__)
    else:
        traceback.print_exc(from_ex)
        raise raise_ex


def node_from_string(yml_str):
    """Return a PyYAML node constructed from the given YAML string."""
    return yaml.compose(yml_str)


def formattypeof(obj):
    """Return the type of the given object as a string without any
    decoration.
    """
    return formattype(type(obj))


def formattype(typeval):
    """Return the given type value as a string without any
    decoration.
    """
    typestring = str(typeval)
    start = typestring.index("'") + 1
    end = typestring.rindex("'")
    return typestring[start:end]


pyge3 = None
"""Whether the user's Python version is greater than or equal to 3."""

if sys.version_info >= (3,):
    pyge3 = True
elif sys.version_info >= (2, 6):
    pyge3 = False
else:
    raise ImportError('Your Python version is not supported. '
            'Please consider upgrading.')

