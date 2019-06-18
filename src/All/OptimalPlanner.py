#!/usr/bin/env python

import config as cfg
from math import sin, cos, tan, pi, sqrt
from functools import partial
from numpy import loadtxt
import matplotlib.pyplot as plt
import sys
import argparse

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import control as oc
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og


def isStateValid(spaceInformation, state):
    return spaceInformation.satisfiesBounds(state)

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj

def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    return opt

def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    else:
        ou.OMPL_ERROR("Optimization-objective is not implemented in allocation function.")


def plan(runTime, plannerType, objectiveType, fname):
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    # X bounds
    bounds.setLow(0, cfg.x_low)
    bounds.setHigh(0, cfg.x_high)
    #Y bounds
    bounds.setLow(1, cfg.y_low)
    bounds.setHigh(1, cfg.y_high)
    space.setBounds(bounds)

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(ob.StateValidityCheckerFn(partial(isStateValid,si)))
    si.setup()

    start = ob.State(space)
    start[0] = cfg.x_pos
    start[1] = cfg.y_pos
    start[2] = cfg.phi

    goal = ob.State(space)
    goal[0] = cfg.wp_x
    goal[1] = cfg.wp_y

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        # Output the length of the path found
        print('{0} found solution of path length {1:.4f} with an optimization ' \
              'objective value of {2:.4f}'.format( \
            optimizingPlanner.getName(), \
            pdef.getSolutionPath().length(), \
            pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))
        print(pdef.getSolutionPath().printAsMatrix())
        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname, 'w') as outFile:
                outFile.write(pdef.getSolutionPath().printAsMatrix())
            outFile.close()
            path = loadtxt(fname)
            plt.plot(path[:, 0], path[:, 1], linestyle="--", marker='>')
            plt.xlabel('x-position [m]')
            plt.ylabel('y-position [m]')
            plt.title('Optimally Planned Path')
            plt.show()
    else:
        print("No solution found.")


if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Optimal motion planning demo program.')

    # Add a filename argument
    parser.add_argument('-t', '--runtime', type=float, default=1.0, help= \
        '(Optional) Specify the runtime in seconds. Defaults to 1 and must be greater than 0.')
    parser.add_argument('-p', '--planner', default='RRTstar', \
                        choices=['BFMTstar', 'BITstar', 'FMTstar', 'InformedRRTstar', 'PRMstar', 'RRTstar', \
                                 'SORRTstar'], \
                        help='(Optional) Specify the optimal planner to use, defaults to RRTstar if not given.')
    parser.add_argument('-o', '--objective', default='PathLength', \
                        choices=['PathClearance', 'PathLength', 'ThresholdPathLength', \
                                 'WeightedLengthAndClearanceCombo'], \
                        help='(Optional) Specify the optimization objective, defaults to PathLength if not given.')
    parser.add_argument('-f', '--file', default="optimalpath.txt", \
                        help='(Optional) Specify an output path for the found solution path.')
    parser.add_argument('-i', '--info', type=int, default=0, choices=[0, 1, 2], \
                        help='(Optional) Set the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG.' \
                             ' Defaults to WARN.')

    # Parse the arguments
    args = parser.parse_args()

    # Check that time is positive
    if args.runtime <= 0:
        raise argparse.ArgumentTypeError(
            "argument -t/--runtime: invalid choice: %r (choose a positive number greater than 0)" \
            % (args.runtime,))

    # Set the log level
    if args.info == 0:
        ou.setLogLevel(ou.LOG_WARN)
    elif args.info == 1:
        ou.setLogLevel(ou.LOG_INFO)
    elif args.info == 2:
        ou.setLogLevel(ou.LOG_DEBUG)
    else:
        ou.OMPL_ERROR("Invalid log-level integer.")

    # Solve the planning problem
    plan(args.runtime, args.planner, args.objective, args.file)
