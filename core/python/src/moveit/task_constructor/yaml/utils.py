#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module contains utility functions and classes for converting to
YAML such as exceptions, Python version checking and type formatting.

This also contains the correct YAML `Loader` and `Dumper` classes
depending on whether LibYAML bindings are available.
"""

from __future__ import print_function

__author__ = 'Jan Ebert'

import sys
import traceback

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
    print('LibYAML is not available. Using slower PyYAML.\n'
            'This may affect compatibility.')


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


def iterdict(obj):
    """Return an iterator yielding the given dictionary's keys and
    values as tuples.
    """
    if pyge3:
        return obj.items()
    else:
        return obj.iteritems()


def attributes_from_dict(obj, dict_, skip_none=False):
    """Set the given object's attributes corresponding to the names and
    values contained in the given dictionary.

    Optionally skip `None`-values.
    """
    for name, value in iterdict(dict_):
        if not skip_none or value is not None:
            setattr(obj, name, value)


pyge3 = None
"""Whether the user's Python version is greater than or equal to 3."""

if sys.version_info >= (3,):
    pyge3 = True
elif sys.version_info >= (2, 6):
    pyge3 = False
else:
    raise ImportError('Your Python version is not supported. '
            'Please consider upgrading.')

