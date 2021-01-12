#! /usr/bin/env python

from __future__ import print_function
import unittest
import rostest
from moveit.python_tools import roscpp_init
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
import rospy
import copy


class TestTrampolines(unittest.TestCase):
    """ Test the functionality of pybind11 trampoline classes.
    - Python classes are able to inherit from C++ base classes, which are
      exposed to python through trampoline classes and bindings.
    - Test for overriding of virtual inheritance of member functions
      and correct construction of the classes.
    """

    def check(self, test_class, **kvargs):
        """Test a trampoline class.
        - Parameter:
            1. test_class: Contains the class to be tested.
            2. kvargs: Contain the parameters to be tested.
        - Tests:
            1. Constructor parameters from kvargs (e.g. "name")
            2. Inheritance structure
        """

        def _checkBases():
            """ Check inheritance stucture.
            - Performed tests:
                1. Number of base classes
            """

            self.assertEqual(
                len(bases),
                1,
                msg="Class '" + type(test_class).__name__ +
                "' must only inherit from a single base class.")

        def _checkParameters():
            """ Check correct assignment of standard parametes.
            - Tests are only executed if the corresponding key value pair
              is given in the kvargs.
            - Possible tests depending on kvargs:
                1. name check
            """

            if "name" in kvargs:
                self.assertEqual(test_class.name, kvargs.get("name"))

        # get the base classes of the derived instance
        bases = test_class.__class__.__bases__

        # perform checks
        _checkParameters()
        _checkBases()

    def test_connecting(self):
        def checkInTask(test_class):
            """ Checks correct insertion into the mtc task hierarchy.
            - The @p test_class should be able to connect two generator stages.
            """
            task = core.Task()
            task.add(stages.CurrentState("current"))
            task.add(test_class)
            task.add(stages.CurrentState("current"))
            task.plan()

        class extConnecting(core.Connecting):
            """ Implements a 'Connecting' stage
            """

            def __init__(self, name):
                core.Connecting.__init__(self, name)

            def compute(self, from_state, to_state):
                print('compute called')

        kvargs = {'name': 'Connecting'}
        extConnecting = extConnecting(kvargs.get('name'))

        # check parameters and base classes
        self.check(extConnecting, **kvargs)

        # check task insertion
        checkInTask(extConnecting)

    def test_generator(self):
        def checkInTask(test_class):
            """ Checks correct insertion into the mtc task hierarchy.
            - The @ p test_class should be able to be inserted as a
              generator into the task hierarchy.
            """
            task = core.Task()
            task.add(test_class)
            task.plan()

        class extGenerator(core.Generator):
            """ Implements a 'Generator' stage.
            """

            def __init__(self, name):
                core.Generator.__init__(self, name)
                self.num = 1

            def reset(self):
                print('RESET')

            def canCompute(self):
                return self.num > 0

            def compute(self):
                self.num -= 1
                print("compute called")

        kvargs = {"name": 'Generator'}
        extGenerator = extGenerator(kvargs.get('name'))
        self.check(extGenerator, **kvargs)

        # check task insertion
        checkInTask(extGenerator)

    @unittest.skip("Monitoring Generator is not yet ready.")
    def test_monitoringGenerator(self):
        def checkInTask(test_class):
            """ Checks correct insertion into the mtc task hierarchy
            - The @ p test_class is tested to be able to ...
                1. ... be inserted as a monitoring generator stage
                2. ... execute the setMonitoredStage Method
            """
            task = core.Task()
            task.add(stages.CurrentState('current'))

            # sanity check
            self.assertEqual(task['current'].name, 'current')

            # add the monitored stage to the monitoring generator
            test_class.setMonitoredStage(task['current'])

            task.add(test_class)
            task.plan()

        class extMonitoringGenerator(core.MonitoringGenerator):
            """ Implements a 'MonitoringGenerator' stage.
            """

            def __init__(self, name):
                core.MonitoringGenerator.__init__(self, name)

            def onNewSolution(self, s):
                print('onNewSolution called')

        kvargs = {'name': 'MonitoringGenerator'}
        extMonitoringGenerator = extMonitoringGenerator(kvargs.get('name'))
        self.check(extMonitoringGenerator, **kvargs)

        # check task insertion
        checkInTask(extMonitoringGenerator)

    def test_propagatingEitherWay(self):
        def checkInTask(test_class):
            """ Checks correct insertion into the mtc task hierarchy.
            - The stage @ p test_class should be able to propagate a generator
              signal into an arbitrary direction.
            """

            task = core.Task()
            task.add(stages.CurrentState('current'))
            task.add(test_class)
            task.plan()

        class extPropagatingEitherWay(core.PropagatingEitherWay):
            """ Implements a 'PropagatingEitherWay' stage.
            """

            def __init__(self, name):
                core.PropagatingEitherWay.__init__(self, name)

            def computeForward(self, from_state):
                print('compute forward')

            def computeBackward(self, to_state):
                print('compute backward')

        kvargs = {'name': 'PropagatingEitherWay'}
        extPropagatingEitherWay = extPropagatingEitherWay(kvargs.get('name'))
        self.check(extPropagatingEitherWay, **kvargs)

        # check task insertion
        checkInTask(extPropagatingEitherWay)


if __name__ == '__main__':
    roscpp_init("test_mtc")
    unittest.main()
