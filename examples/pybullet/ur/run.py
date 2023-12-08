#!/usr/bin/env python

from __future__ import print_function

# utility modules
import sys
import signal
from itertools import count

#PDDLStream related modules
from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test, empty_gen, universe_test
from pddlstream.algorithms.focused import solve_focused

from examples.pybullet.utils.pybullet_tools.utils import HideOutput, LockRenderer, wait_for_user

#MoveIt related imports
from math import tau
from osx_robot_control.core import OSXCore
import rospy
from ur_control import conversions


# primitives

class BodyConf(object):
    num = count()
    def __init__(self, osx, configuration = None):
        if configuration is None:
            configuration = osx.a_bot.robot_group.get_current_joint_values()
        self.configuration = configuration
        self.index = next(self.num)
    def values(self):
        return self.configuration
    def __repr__(self):
        index = self.index
        #return 'q{}'.format(id(self) % 1000)
        return 'q{}'.format(index)
    
class ObjectPose(object):
    num = count()
    def __init__(self, object, pose=None):
        self.object = object
        self.pose = pose
        if pose is None:
            pose = [0.0, -0.22, 0.27, tau/4, tau/4, 0] # placeholder
        #self.value = tuple(value)
        #self.init = init
        self.index = next(self.num)
    def value(self):
        return self.pose
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'p{}'.format(index)
        
class Command(object):
    num = count()
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.index = next(self.num)
    def __repr__(self):
        index = self.index
        return 'c{}'.format(index)


# Stream implementations 

def get_ik_fn(osx): 
    def fn(object_name, pose): # receives the object name and pose, outputs a cartesian trajectory
        target_pose = pose.value()
        print(target_pose)
        pose_goal = conversions.to_pose_stamped(frame_id="cutting_board", pose=target_pose)
        trajectory = osx.a_bot.go_to_pose_goal(pose_goal_stamped=pose_goal, speed=0.3)
        if trajectory is None:
            print('Approach path failure')
        print(trajectory)
        #conf = BodyConf
        #conf = BodyConf(osx, end_conf)
        command = Command(trajectory)
        return (command)
    return fn


# Defining the PDDLStream problems

def pddlstream_from_problem(osx):

    # get domain & stream files
    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    init = [ ('CanMove',),]

    arm = 'UR'
    current_joint_values = osx.a_bot.robot_group.get_current_joint_values()
    conf = BodyConf(osx, current_joint_values)
    init += [('AtConf', conf)]


    obj_name = 'cutting_board'
    obj_pose = [0.0, -0.22, 0.27, tau/4, tau/4, 0] # pre-specify pose goal values for now
    #pose_goal = conversions.to_pose_stamped
    pose = ObjectPose(obj_name, obj_pose)
    init += [
        ('AtPose', obj_name, pose),
    ]

    # Goal state : arm must move to the object pose

    goal = (
        ('AtPose', obj_name, pose)
        #('move', arm, pose), # derived action?
    )

    # maps the stream.pddl functions to real functions
    stream_map = {

        'inverse-kinematics': from_gen_fn(get_ik_fn(osx)),
    }

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


# main function
def main():

    rospy.init_node("UR PDDLStream")
    osx = OSXCore()

    parser = create_parser()
    args = parser.parse_args()

    problem = pddlstream_from_problem(osx)
    #print(problem)

    _, _, _, stream_map, init, goal = problem  

    print('Init: ', init)
    print('Goal: ', goal)
    print('Streams', str_from_object(set(stream_map)))

    with Profiler():
        solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit, success_cost=INF)

    print_solution(solution)
    plan, _, _, = solution
    if plan is None:
        return
    
    print(plan)
    

if __name__ == '__main__':
    main()




