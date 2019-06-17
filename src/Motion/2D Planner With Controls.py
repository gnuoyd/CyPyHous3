#!/usr/bin/env python

import config as cfg
from math import sin, cos, tan, pi
from functools import partial
from numpy import loadtxt
import matplotlib.pyplot as plt

try:
    from ompl import base as ob
    from ompl import control as oc
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import control as oc


#class MyDecomposition(oc.GridDecomposition):
    #def __init__(self, length, bounds):
        #super(MyDecomposition, self).__init__(length, 2, bounds)
    #def project(self, s, coord):
        #coord[0] = s.getX()
        #coord[1] = s.getY()
    #def sampleFullState(self, sampler, coord, s):
        #sampler.sampleUniform(s)
        #s.setXY(coord[0], coord[1])


def isStateValid(spaceInformation, state):
    return spaceInformation.satisfiesBounds(state)


def propagate(start, control, duration, state):
    state.setX(start.getX() + control[0] * duration * cos(start.getYaw()))
    state.setY(start.getY() + control[0] * duration * sin(start.getYaw()))
    state.setYaw(start.getYaw() + control[0] * tan(control[1]) * duration / cfg.carlength)


def plan():
    # Construct the state space we are planning in
    space = ob.SE2StateSpace()

    # Set the bounds for the R^2 part of SE(2) based on arena dimensions
    bounds = ob.RealVectorBounds(2)
    # X bounds
    bounds.setLow(0, cfg.x_low)
    bounds.setHigh(0, cfg.x_high)
    #Y bounds
    bounds.setLow(1, cfg.y_low)
    bounds.setHigh(1, cfg.y_high)
    space.setBounds(bounds)

    # Create a control space
    cspace = oc.RealVectorControlSpace(space, 2)

    # Set the bounds for the control space
    cbounds = ob.RealVectorBounds(2)
    #Velocity bounds
    cbounds.setLow(0, cfg.v_low)
    cbounds.setHigh(0, cfg.v_high)
    #Steering angle bounds
    cbounds.setLow(1, cfg.delta_low)
    cbounds.setHigh(1, cfg.delta_high)
    cspace.setBounds(cbounds)

    # define a simple setup class
    ss = oc.SimpleSetup(cspace)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation())))
    ss.setStatePropagator(oc.StatePropagatorFn(propagate))

    # Create a start state
    start = ob.State(space)
    start().setX(cfg.x_pos)
    start().setY(cfg.y_pos)
    start().setYaw(cfg.phi)

    # Create a goal state
    goal = ob.State(space)
    goal().setX(cfg.wp_x)
    goal().setY(cfg.wp_y)
    #goal().setYaw(0.0)

    # Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    # Set planner
    si = ss.getSpaceInformation()
    planner = oc.SST(si)
    # planner = oc.EST(si)
    #planner = oc.KPIECE1(si) # this is the default
    # SyclopEST and SyclopRRT require a decomposition to guide the search
    #decomp = MyDecomposition(32, bounds)
    #planner = oc.SyclopEST(si, decomp)
    #planner = oc.SyclopRRT(si, decomp)
    ss.setPlanner(planner)
    # (optionally) set propagation step size
    si.setPropagationStepSize(0.1)
    si.setMinMaxControlDuration(5,10)
    # Attempt to solve the problem
    solved = ss.solve(5.0)

    if solved:
        # print the path to screen
        print("Found solution:\n%s" % ss.getSolutionPath().printAsMatrix())
        data = open("planned_path.txt", 'w')
        data.write(ss.getSolutionPath().printAsMatrix())
        data.close()
        path = loadtxt('planned_path.txt')
        plt.plot(path[:, 0], path[:, 1], linestyle="--", marker='>')
        plt.xlabel('x-position [m]')
        plt.ylabel('y-position [m]')
        plt.title('SST Planned Path')
        plt.show()


if __name__ == "__main__":
    plan()
