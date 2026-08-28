from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
import math


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    position, haskit, pendingSystems = state

    if not haskit:
        objective = problem.kitPosition
    elif len(pendingSystems) != 0:
        objective = min(pendingSystems, key=lambda x: abs(position[0] - x[0]) + abs(position[1] - x[1]))
    else:
        objective = problem.controlPosition
    return abs(position[0] - objective[0]) + abs(position[1] - objective[1])



def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    position, haskit, pendingSystems = state
    
    if not haskit:
        objective = problem.kitPosition
    elif len(pendingSystems) != 0:
        objective = min(pendingSystems, key=lambda x: math.sqrt((position[0] - x[0]) ** 2 + (position[1] - x[1]) ** 2))
    else:
        objective = problem.controlPosition
    return math.sqrt((position[0] - objective[0]) ** 2 + (position[1] - objective[1]) ** 2)


def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    position, hasKit, pendingSystems = state
    retorno = None

    if not hasKit:
        tramo1 = distanciaManhattan(position, problem.kitPosition)
        mas_lejano = max(pendingSystems, key=lambda t: distanciaManhattan(problem.kitPosition, t) + distanciaManhattan(t, problem.controlPosition))
        tramo2 = distanciaManhattan(problem.kitPosition, mas_lejano) + distanciaManhattan(mas_lejano, problem.controlPosition)
        return tramo1 + tramo2

    elif pendingSystems:
        mas_lejano = max(pendingSystems, key=lambda t: distanciaManhattan(position, t) + distanciaManhattan(t, problem.controlPosition))
        tramo = distanciaManhattan(position, mas_lejano) + distanciaManhattan(mas_lejano, problem.controlPosition)
        return tramo

    else:
        retorno = distanciaManhattan(position, problem.controlPosition)
        
    return retorno

def distanciaManhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
