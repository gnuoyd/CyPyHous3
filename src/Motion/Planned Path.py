import sys
from functools import partial
from typing import Union

from geometry_msgs.msg import Pose

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

# Arena bounds [m]
x_low = -2.5
x_high = 2.5
y_low = -3.5
y_high = 3.5


class Planner(object):

    def __init__(self, motionautomaton):
        self.__motionautomaton = motionautomaton

    def isStateValid(self, spaceInformation, state):
        return spaceInformation.satisfiesBounds(state)

    def allocatePlanner(self, si, plannerType):
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

    def planned_path(self, curr_loc: tuple, waypoint: tuple, runTime: float = 1.0, plannerType: str = "rrtstar") -> \
    Union[list, None]:
        # Construct the robot state space in which we're planning. We're
        # planning in [0,1]x[0,1], a subset of R^2.
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        # X bounds
        bounds.setLow(0, x_low)
        bounds.setHigh(0, x_high)
        # Y bounds
        bounds.setLow(1, y_low)
        bounds.setHigh(1, y_high)
        space.setBounds(bounds)

        # Construct a space information instance for this state space
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(partial(self.isStateValid, si)))
        si.setup()

        start = ob.State(space)
        start[0] = curr_loc[0]
        start[1] = curr_loc[1]

        goal = ob.State(space)
        goal[0] = waypoint[0]
        goal[1] = waypoint[1]

        # Create a problem instance
        pdef = ob.ProblemDefinition(si)

        # Set the start and goal states
        pdef.setStartAndGoalStates(start, goal)
        # Create the optimization objective specified by our command-line argument.
        # This helper function is simply a switch statement.
        pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

        # Construct the optimal planner specified by our command line argument.
        # This helper function is simply a switch statement.
        optimizingPlanner = self.allocatePlanner(si, plannerType)

        # Set the problem instance for our planner to solve
        optimizingPlanner.setProblemDefinition(pdef)
        optimizingPlanner.setup()

        # attempt to solve the planning problem in the given runtime
        solved = optimizingPlanner.solve(runTime)
        if solved:
            path = pdef.getSolutionPath()
            return path
        else:
            return None

    def follow_path(self, path: list) -> None:
        path1 = path[1:, :]
        wp_list = []
        for row in range(len(path1)):
            wp = Pose()
            wp.position.x = path1[row][0]
            wp.position.y = path1[row][1]
            wp_list.append(wp)
        for wp in wp_list[:-1]:
            self.__motionautomaton.goTo(wp, 1)
        self.__motionautomaton.goTo(wp_list[-1], 0)
