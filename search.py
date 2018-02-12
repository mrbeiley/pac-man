# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util #contains data structures

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).

  You do not need to change anything in this class, ever.
  """

  def getStartState(self):
     """
     Returns the start state for the search problem
     """
     util.raiseNotDefined()

  def isGoalState(self, state):
     """
       state: Search state

     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state

     For a given state, this should return a list of triples,
     (successor, action, stepCost), where 'successor' is a
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take

     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def graphSearch(problem, strategy):
    """
    problem: initializes board state
    strategy: type of ordering structure for search
    """
    start_node = Node(problem.getStartState(), None , 0, None)
    strategy.push(start_node)
    explored = set()

    while strategy.isEmpty() == False:
        node = strategy.pop()
        if node.state in explored: continue
        if problem.isGoalState(node.state) == True:
            return node.getSolution(None, None)
        else:
            explored.add(node.state)

            for succ in problem.getSuccessors(node.state):
                succ_node = Node(succ[0],succ[1],succ[2], node)
                if succ_node.state not in explored and succ_node not in strategy.list:
                    strategy.push(succ_node)


def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].

  uses graphSearch with a stack (LIFO) strategy
  """
  return graphSearch(problem, util.Stack())

def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first. [p 81]

  uses graphSearch with a queue (FIFO) strategy
  """
  return graphSearch(problem, util.Queue())

def uniformCostSearch(problem):
  """Search the node of least total cost first.

  operates similar to graphSearch, but with
  extra cost checking functionality
  """
  strategy = util.PriorityQueue()
  start_node = Node(problem.getStartState(), None , 0, None)
  strategy.push(start_node, start_node.cost)
  explored = set()

  while strategy.isEmpty() == False:
      node = strategy.pop()
      if node.state in explored: continue
      if problem.isGoalState(node.state) == True:
          return node.getSolution()
      else:
          explored.add(node.state)
          for succ in problem.getSuccessors(node.state):
              succ_node = Node(succ[0],succ[1],node.cost+ succ[2], node)
              if succ_node.state not in explored and succ_node not in strategy.heap:
                  strategy.push(succ_node, succ_node.cost)
              elif succ_node in strategy.heap:
                  compare_node = strategy.find_and_extract(succ_node.state)
                  if succ_node.cost < compare_node.cost:
                      strategy.push(succ_node, succ_node.cost)
                  else:
                      strategy.push(compare_node, compare_node.cost)


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.

    """
    heur_start = heuristic(problem.getStartState(), problem)
    strategy = util.PriorityQueue()
    start_node = Node(problem.getStartState(), None , 0, None)
    strategy.push(start_node, heuristic(start_node.state, problem) +start_node.cost)
    explored = set()

    while strategy.isEmpty() == False:
        node = strategy.pop()
        if node.state in explored: continue
        if problem.isGoalState(node.state) == True:
          return node.getSolution(heuristic, problem)
        else:
          explored.add(node.state)
          for succ in problem.getSuccessors(node.state):
              succ_node = Node(succ[0],succ[1],node.cost+ succ[2], node)
              if succ_node.state not in explored and succ_node not in strategy.heap:
                  strategy.push(succ_node, succ_node.cost + heuristic(succ_node.state, problem))

              elif succ_node in strategy.heap:
                  compare_node = strategy.find_and_extract(succ_node.state)
                  if succ_node.cost < compare_node.cost:
                      strategy.push(succ_node, succ_node.cost + heuristic(succ_node.state, problem))
                  else:
                      strategy.push(compare_node, compare_node.cost + heuristic(compare_node.state, problem))




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

#shorthand directons
from game import Directions
s = Directions.SOUTH
w = Directions.WEST
n = Directions.NORTH
e = Directions.EAST

class Node():
    """
    Node class is a full representation of a node in a search graph, with attributes:

    *state encodes all the necessary components for the problem to understand the world (subset of Pacman gameState)
    *action is the move necessary to reach the node from its parent
    *cost is the full cost of the path from start to the node
    *parent node necessary to traverse back up the tree to get a solution path
    """

    def __init__(self, state, action, cost, parent):

        self.state = state
        if(action == 'North'):
            self.action = n
        elif(action == 'South'):
            self.action = s
        elif(action == 'West'):
            self.action = w
        elif(action =='East'):
            self.action = e
        else:
            self.action = None
        self.cost = cost
        self.parent = parent


    def getSolution(self, heuristic, problem):
        """


        """
        path = [self.action]
        parent = self.parent
        while parent.action != None:
            path.append(parent.action)
            parent = parent.parent
            print parent.cost + heuristic(parent.state, problem)
        #for i in xrange(len(self.path)):
        #        if(self.path[i] == 'North'):
        #            self.path[i] = n
        #        elif(self.path[i] == 'South'):
        #            self.path[i] = s
        #        elif(self.path[i] == 'West'):
        #            self.path[i] = w
        #        else:
        #            self.path[i] = e
        return path[::-1]
