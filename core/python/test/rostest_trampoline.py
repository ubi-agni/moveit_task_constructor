#! /usr/bin/env python

from __future__ import print_function
import unittest
import mock
import rostest
from moveit.python_tools import roscpp_init
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
import rospy
import copy


class TestTrampolines(unittest.TestCase):
    """ Test the functionality of trampoline classes.
    - Python classes are able to inherit from C++ base classes, which are
      exposed to python through trampoline classes and bindings.
    - Test for overriding of virtual inheritance of member functions
      and correct construction of the classes.
    """

    def setUp(self):
        roscpp_init("test_mtc")

    def check(self, test_class, **kwargs):
        """Test a trampoline class.
        - Parameter:
            1. test_class: Contains the class to be tested.
            2. kwargs: Contain the parameters to be tested.
        - Tests:
            1. Constructor parameters from kwargs (e.g. "name")
            2. Inheritance structure
        """

        def _checkBases(test_class):
            """ Check inheritance stucture.
            - Performed tests:
                1. Number of base classes
            """

            self.assertEqual(
                len(test_class.__class__.__bases__),
                1,
                msg="Class '" + type(test_class).__name__ +
                "' must only inherit from a single base class.")

        def _checkParameters(test_class):
            """ Check correct assignment of standard parametes.
            - Tests are only executed if the corresponding key value pair
              is given in the kwargs.
            - Possible tests depending on kwargs:
                1. name check
            """

            if "name" in kwargs:
                self.assertEqual(test_class.name, kwargs.get("name"))

        # actually call the checks
        _checkParameters(test_class)
        _checkBases(test_class)

    def test_connecting(self):
        class extConnecting(core.Connecting):
            """ Implements a 'Connecting' stage
            """

            def __init__(self, name):
                core.Connecting.__init__(self, name)

            def compute(self, from_state, to_state):
                pass

        kwargs = {'name': 'Connecting'}
        connecting = extConnecting(kwargs.get('name'))

        # check parameters and base classes
        self.check(connecting, **kwargs)

        with mock.patch.object(extConnecting, 'compute') as compute:
            task = core.Task()
            task.add(stages.CurrentState("current"))
            task.add(connecting)
            task.add(stages.CurrentState("current"))
            task.plan()
            task.reset()

            compute.assert_called_once()

    def test_generator(self):
        class extGenerator(core.Generator):
            """ Implements a 'Generator' stage.
            """

            def __init__(self, name):
                core.Generator.__init__(self, name)
                self.num = self.computeCalls = 3

            def reset(self):
                pass

            def canCompute(self):
                return self.num > 0

            def compute(self):
                self.num -= 1

        kwargs = {"name": 'Generator'}
        generator = extGenerator(kwargs.get('name'))
        self.check(generator, **kwargs)

        with mock.patch.object(extGenerator, 'canCompute', wraps=generator.canCompute) as canCompute, \
                mock.patch.object(extGenerator, 'compute', wraps=generator.compute) as compute:

            task = core.Task()
            task.add(generator)
            task.plan()
            task.reset()

            canCompute.assert_called()

            # canCompute should be called computeCount * 2 + 1 times:
            #   - check inside stage itself and it's wrapping container
            self.assertEqual(
                canCompute.call_count,
                generator.computeCalls * 2 + 1)

            compute.assert_called()
            self.assertEqual(compute.call_count, generator.computeCalls)

    @unittest.skip("Monitoring Generator is not yet ready.")
    def test_monitoringGenerator(self):
        class extMonitoringGenerator(core.MonitoringGenerator):
            """ Implements a 'MonitoringGenerator' stage.
            """

            def __init__(self, name):
                core.MonitoringGenerator.__init__(self, name)

            def onNewSolution(self, s):
                print('onNewSolution called')

        kwargs = {'name': 'MonitoringGenerator'}
        monitoringGenerator = extMonitoringGenerator(kwargs.get('name'))
        self.check(monitoringGenerator, **kwargs)

        with mock.patch.object(extMonitoringGenerator, 'onNewSolution', wraps=monitoringGenerator.onNewSolution) as onNewSolution:

            task = core.Task()
            task.add(stages.CurrentState('current'))

            # sanity check
            self.assertEqual(task['current'].name, 'current')

            # add the monitored stage to the monitoring generator
            monitoringGenerator.setMonitoredStage(task['current'])

            task.add(monitoringGenerator)
            task.plan()
            task.reset()

            onNewSolution.assert_called()

    def test_propagatingEitherWay(self):
        class extPropagatingEitherWay(core.PropagatingEitherWay):
            """ Implements a 'PropagatingEitherWay' stage.
            """

            def __init__(self, name):
                core.PropagatingEitherWay.__init__(self, name)

            def computeForward(self, from_state):
                pass

            def computeBackward(self, to_state):
                pass

        kwargs = {'name': 'PropagatingEitherWay'}

        propagatingForward = extPropagatingEitherWay(kwargs.get('name'))
        self.check(propagatingForward, **kwargs)

        propagatingBackward = extPropagatingEitherWay(kwargs.get('name'))
        self.check(propagatingBackward, **kwargs)

        with mock.patch.object(extPropagatingEitherWay, 'computeForward', wraps=propagatingForward.computeForward) as computeForward, \
                mock.patch.object(extPropagatingEitherWay, 'computeBackward', wraps=propagatingBackward.computeBackward) as computeBackward:

            # check compute forward
            task = core.Task()
            task.add(stages.CurrentState('current'))
            task.add(propagatingForward)
            task.plan()
            task.reset()
            computeForward.assert_called_once()
            computeBackward.assert_not_called()

        with mock.patch.object(extPropagatingEitherWay, 'computeForward', wraps=propagatingBackward.computeForward) as computeForward, \
                mock.patch.object(extPropagatingEitherWay, 'computeBackward', wraps=propagatingBackward.computeBackward) as computeBackward:

            # check compute Backward
            task = core.Task()
            task.add(propagatingBackward)
            task.add(stages.CurrentState('current'))
            task.plan()
            task.reset()
            computeBackward.assert_called_once()
            computeForward.assert_not_called()


if __name__ == '__main__':
    unittest.main()
