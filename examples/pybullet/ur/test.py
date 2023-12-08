#!/usr/bin/env python

from __future__ import print_function

# utility modules
import sys
import signal

#PDDLStream related modules
from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test, empty_gen, universe_test

from examples.pybullet.utils.pybullet_tools.utils import HideOutput, LockRenderer, wait_for_user


#MoveIt related imports
from math import tau
from osx_robot_control.core import OSXCore()
import rospy
from ur_control import conversions


class Conf(object):
    def __init__(self, osx, configuration = None):
        if configuration is None:
            configuration = osx.a_bot.robot_group.get_current_joint_values()
        self.configuration = configuration
    def values(self):
        return self.configuration
    def __repr__(self):
        index = self.index
        return 'q{}'.format(index)

class ObjectPose(object):


def main():
    rospy.init_node("UR PDDLStream")
    osx = OSXCore()

    current_joint_values = osx.a_bot.robot_group.get_current_joint_values()
    conf = Conf(osx, current_joint_values)
    print(conf)

if __name__ == '__main__':
    main()


