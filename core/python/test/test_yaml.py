#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module supports tests for YAML conversion and deconversion.

Each class has a module, name (in plural) and verbosity attribute that
is used for generating tests and error messages.
"""

from __future__ import print_function

__author__ = 'Jan Ebert'

import unittest

import yaml
from moveit_msgs.msg import JointConstraint, MoveGroupGoal

from moveit.task_constructor.yaml import rosmsg, stage
from moveit.task_constructor import core, stages


def _dump_and_reconstruct(test, orig_obj, verbose=None):
    """Dump and reconstruct an object, then test whether the pre- and
    post-conversion objects differ.

    Override test class verbosity option with the extra parameter.
    """
    if not isinstance(test, unittest.TestCase):
        raise TypeError('Please make sure the first argument is derived from '
                'unittest.TestCase')
    if verbose is None:
        verbose = test.verbose

    yml = test.module.toyaml(orig_obj)
    if verbose:
        print(yml)

    recons_obj = test.module.fromyaml(yml)  # reconstructed
    if isinstance(orig_obj, core.Stage):  # testing for equality does not work
        test.assertEqual(orig_obj.name, recons_obj.name)
        orig_props = orig_obj.properties
        recons_props = recons_obj.properties

        # compare size of properties (do not trust sys.sizeof() for user types)
        orig_len = 0
        recons_len = 0
        for _ in orig_props:
            orig_len += 1
        for _ in recons_props:
            recons_len += 1
        test.assertEqual(orig_len, recons_len)

        for name, value in orig_obj.properties:
            test.assertEqual(value, recons_props[name])
    else:
        test.assertEqual(orig_obj, recons_obj,
                ('{} are not equal.\n----- Original: -----\n{}\n\n'
                '----- Reconstructed: -----\n{}').format(
                test.name, orig_obj, recons_obj))


class TestMsgs(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestMsgs, self).__init__(*args, **kwargs)
        self.module = rosmsg
        self.name = 'Messages'
        self.verbose = False

    def test_JointConstraint(self):
        _dump_and_reconstruct(self, JointConstraint('test', 1, 2, 3, 4))

    def test_MoveGroupGoal(self):
        _dump_and_reconstruct(self, MoveGroupGoal())


class TestStages(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStages, self).__init__(*args, **kwargs)
        self.module = stage
        self.name = 'Stages'
        self.verbose = False

    def test_FixedState(self):
        _dump_and_reconstruct(self, stages.FixedState('fixed'))

    def test_Connect(self):
        planner = core.PipelinePlanner()
        planner2 = core.PipelinePlanner()
        stage = stages.Connect('connect',
                [('planner', planner), ('planner2', planner2)])
        _dump_and_reconstruct(self, stage)


if __name__ == '__main__':
    unittest.main()

