#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""This module supports tests for YAML conversion and deconversion.

Each class has a name (in plural) and verbosity attribute which are used
to generate error messages.
"""

from __future__ import print_function

__author__ = 'Jan Ebert'

import unittest

from moveit_msgs.msg import JointConstraint, MoveGroupGoal

from moveit.task_constructor.yaml import toYaml, fromYaml
from moveit.task_constructor import core, stages


class TestHelper(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestHelper, self).__init__(*args, **kwargs)
        self.verbose = False

    def _dump_and_reconstruct(self, original, verbose=None):
        """Dump and reconstruct an object, then test whether the pre- and
        post-conversion objects differ.
        """
        if verbose is None:
            verbose = self.verbose

        yml = toYaml(original)
        if verbose:
            print(yml)

        restored = fromYaml(yml)  # reconstructed
        if isinstance(original, core.Stage) or isinstance(original, core.Task):  # testing for equality does not work
            try:
                self.assertEqual(original.name, restored.name)
            except AttributeError:
                pass

            for name, value in original.properties:
                self.assertEqual(value, restored.properties[name])
        else:
            self.assertEqual(original, restored,
                    ('{} are not equal.\n----- Original: -----\n{}\n\n'
                    '----- Restored: -----\n{}').format(
                    self.name, original, restored))


class TestMsgs(TestHelper):
    def __init__(self, *args, **kwargs):
        super(TestMsgs, self).__init__(*args, **kwargs)
        self.name = 'Messages'

    def test_JointConstraint(self):
        self._dump_and_reconstruct(JointConstraint('test', 1, 2, 3, 4))

    def test_MoveGroupGoal(self):
        self._dump_and_reconstruct(MoveGroupGoal())


class TestStages(TestHelper):
    def __init__(self, *args, **kwargs):
        super(TestStages, self).__init__(*args, **kwargs)
        self.name = 'Stages'

    def test_FixedState(self):
        self._dump_and_reconstruct(stages.FixedState('fixed'))

    def test_Task(self):
        task = core.Task()
        task.add(stages.CurrentState("current"))
        task.add(stages.Connect("connect", []))
        task.add(stages.FixedState())
        self._dump_and_reconstruct(task)

    def test_Connect(self):
        planner = core.PipelinePlanner()
        planner2 = core.PipelinePlanner()
        stage = stages.Connect('connect',
                [('planner', planner), ('planner2', planner2)])
        self._dump_and_reconstruct(stage)


if __name__ == '__main__':
    unittest.main()
