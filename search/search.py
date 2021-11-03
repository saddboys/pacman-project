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
import time

import util

class SearchProblem:


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

    def expand(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (child,
        action, stepCost), where 'child' is a child to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that child.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        """
          state: Search state
          action: action taken at state.
          next_state: next Search state after taking action.

        For a given state, this should return the cost of the (s, a, s') transition.
        """
        util.raiseNotDefined()

    def getNextState(self, state, action):
        """
          state: Search state
          action: action taken at state

        For a given state, this should return the next state after taking action from state.
        """
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    mystack = util.Stack()
    startNode = (problem.getStartState(), '', 0, [])
    mystack.push(startNode)
    visited = set()
    while mystack :
        node = mystack.pop()
        state, action, cost, path = node
        if state not in visited :
            visited.add(state)
            if problem.isGoalState(state) :
                path = path + [(state, action)]
                break;
            succNodes = problem.expand(state)
            for succNode in succNodes :
                succState, succAction, succCost = succNode
                newNode = (succState, succAction, cost + succCost, path + [(state, action)])
                mystack.push(newNode)
    actions = [action[1] for action in path]
    del actions[0]
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    starting_state = {
        "state": problem.getStartState(),  # the current state node
        "action": [],
    }

    unvisited_states = []
    visited_states = []
    current_state = starting_state
    while not problem.isGoalState(current_state['state']):


        # append expanded states to open list
        if not any(current_state['state'] == visited_state['state'] for visited_state in visited_states):
            expanded_states = problem.expand(current_state['state'])
            visited_states.append(current_state)
            for expansion in expanded_states:
                child, action, cost = expansion
                # check if state is already visited
                if not any(child == visited_state['state'] for visited_state in visited_states):
                    new_state = {
                        "state": child,  # the current state node
                        "action": current_state['action'].copy(),
                    }
                    new_state['action'].append(action)
                    unvisited_states.append(new_state)

        current_state = unvisited_states.pop(0)

    return current_state['action']

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    open_list = []
    closed_list = []

    start_state = {
        "state": problem.getStartState(),
        "g_value": 0,
        "h_value": heuristic(problem.getStartState(), problem),
        "f_value": heuristic(problem.getStartState(), problem),
        "actions": []
        }
    open_list.append(start_state)
    current_state = None
    while len(open_list) > 0:
        lowest_f = float("inf")

        for state in open_list:
            f_value = state["f_value"]
            if f_value < lowest_f:
                lowest_f = f_value
                current_state = state

        if problem.isGoalState(current_state['state']):
            break

        expanded_states = problem.expand(current_state['state'])
        open_list.remove(current_state)
        closed_list.append(current_state)

        for expanded_state in expanded_states:
            child, action, cost = expanded_state
            g_value = current_state["g_value"] + cost
            new_actions = current_state["actions"].copy()
            new_actions.append(action)
            h_value = heuristic(child, problem)
            new_state = {
                "state": child,
                "g_value": g_value,
                "actions": new_actions,
                "h_value": h_value,
                "f_value": h_value + g_value,
            }

            if any(new_state['state'] == open_state['state'] and
                   open_state['f_value'] < new_state['f_value'] for open_state in open_list):
                continue

            if not any(new_state['state'] == closed_state['state'] for closed_state in closed_list):
                open_list.append(new_state)




    return current_state['actions']

        
def recursivebfs(problem, heuristic=nullHeuristic) :
    #Recursive BFS

    def rbfs(node, f_limit):

        if problem.isGoalState(node["node"]):
            return True, f_limit, node['actions']

        successors = []
        expanded_nodes = problem.expand(node['node'])

        for expanded_node in expanded_nodes:
            child, action, cost = expanded_node

            child_node = {
                "node": child,
                "actions": node['actions'].copy(),
                "g_value": cost + node['g_value']
            }
            child_node["actions"].append(action)
            successors.append(child_node)

        if len(successors) == 0:
            return False, float('inf'), None

        for successor in successors:
            successor_f = heuristic(successor['node'], problem) + successor['g_value']
            successor['f_value'] = max([successor_f, node['f_value']])

        # sort successors by f-value
        while True:
            successors = sorted(successors, key=lambda item: item['f_value'])
            if successors[0]['f_value'] > f_limit:
                return False, successors[0]['f_value'], None

            solved, best_f, actions = rbfs(successors[0], min([f_limit, successors[1]['f_value']]))
            successors[0]['f_value'] = best_f
            if solved:
                return True, successors[0]['f_value'], actions

    start_state = problem.getStartState()
    start_node = {
        "node": start_state,
        "actions": [],
        "g_value": 0,
        "f_value": heuristic(start_state, problem)
    }

    _, _, return_actions = rbfs(start_node, float('inf'))

    return return_actions



    
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
rebfs = recursivebfs
