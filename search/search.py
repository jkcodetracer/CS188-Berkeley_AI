# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "St3rt:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from game import Directions
    s = Directions.STOP

    print "St3rt:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    closed = []
    S = util.Stack()
    v = [problem.getStartState(),[]]
    S.push(v)
    while not S.isEmpty():
        tup = S.pop()
        current = tup[0]
        path = tup[1]
        if problem.isGoalState(current):
            return path

        succesors = problem.getSuccessors(current)
        for node in succesors:
            tmppath = list(path)
            coord = node[0]
            nextstep = node[1]

            if not coord in closed:
                closed.append(coord)
                tmppath.append(nextstep)
                S.push((coord, tmppath))

    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    Q = util.Queue()
    closed = []
    v = [problem.getStartState(), []]
    Q.push(v)
    while not Q.isEmpty():
        tup = Q.pop()
        current = tup[0]
        path = tup[1]
        if problem.isGoalState(current):
            return path

        succesors = problem.getSuccessors(current)
        for node in succesors:
            tmppath = list(path)
            coord = node[0]
            nextstep = node[1]

            if not coord in closed:
                closed.append(coord)
                tmppath.append(nextstep)
                Q.push((coord, tmppath))
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    PQ = util.PriorityQueue()
    closed = []
    v = [problem.getStartState(), []]
    PQ.push(v, 0)
    while not PQ.isEmpty():
        tup = PQ.pop()
        current = tup[0]
        path = tup[1]
        if problem.isGoalState(current):
            return path
        succesors = problem.getSuccessors(current)
        for node in succesors:
            tmppath = list(path)
            coord = node[0]
            nextstep = node[1]
            weight = node[2]

            if not coord in closed:
                closed.append(coord)
                tmppath.append(nextstep)
                PQ.push((coord, tmppath), weight)
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarPath(came_from, current_node, start):
    if start == current_node:
        return []
    if current_node in came_from:
        path = aStarPath(came_from, came_from[current_node][0], start)
        return path + [came_from[current_node][1]]
    else:
        return [came_from[current_node][1]]

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    PQ = util.PriorityQueue()
    start = tuple(problem.getStartState())
    PQ.push(start, 0)
    came_from = {}
    cost_so_far = {}

    came_from[start] = [start, Directions.STOP]
    cost_so_far[start] = 0

    while not PQ.isEmpty():
        current = PQ.pop()
        if problem.isGoalState(current):
            path = aStarPath(came_from, current, start)
            print path
            return path

        succesors = problem.getSuccessors(current)
        for node in succesors:
            coord = tuple(node[0])
            move = node[1]
            weight = node[2]
            new_cost = cost_so_far[current] + weight
            if coord not in cost_so_far or \
                new_cost < cost_so_far[coord]:
                cost_so_far[coord] = new_cost
                new_cost = new_cost + heuristic(coord, problem)
                came_from[coord] = (current, move)
                PQ.push(coord, new_cost)

    return []
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
