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

import util

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

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())-Start's successors: [((34, 15), 'South', 1), ((33, 16), 'West', 1)]

  """
  "*** YOUR CODE HERE ***"
  ###following the pattern of the algorithem in page 82 (but for DFS)
  node=problem.getStartState() # the start of the map
  solution=[]
  frontier=util.Stack()
  frontier.push((node,'Spawn',0))# pushing the spawn location to the frontier
  explored={} # an empty set
  explored[node]='Spawn'
  OrderedPath={} # so we can trace back and show the order of the scan

  if problem.isGoalState(node): #checking if we spawned at the target
      return solution

  targetReached=False

  while(frontier and not targetReached): # while we are not in the end and we still have options to scan

      node=frontier.pop()
      explored[node[0]]=node[1] # put the node and his direction in the explored set

      if problem.isGoalState(node[0]):
          targetNode=node[0]
          targetReached=True
          break
      ###if we didnt arrive to the end point will expand the nodes and check his actions
      for action in problem.getSuccessors(node[0]):

          if action[0] not in explored.keys(): # check if we already did the action
              OrderedPath[action[0]]=node[0]
              frontier.push(action)

  while(targetNode in OrderedPath.keys()): # if we got a solution lets find the path to it

      lastNode=OrderedPath[targetNode]
      solution.insert(0,explored[targetNode])
      targetNode=lastNode # go to the node before
  return solution


  util.raiseNotDefined()

def breadthFirstSearch(problem):
    # following the same logic as BFS from the book and inspired by the DFS
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"
  node = problem.getStartState()  # the start of the map
  solution = []
  frontier = util.Queue()
  frontier.push((node, 'Spawn', 0))  # pushing the spawn location to the frontier
  explored = {}  # an empty set
  explored[node] = 'Spawn'
  OrderedPath = {}  # so we can trace back and show the order of the scan

  if problem.isGoalState(node):  # checking if we spawned at the target
      return solution

  targetReached = False

  while (frontier and not targetReached):  # while we are not in the end and we still have options to scan

      node = frontier.pop()
      explored[node[0]] = node[1]  # put the node and his direction in the explored set

      if problem.isGoalState(node[0]):
          targetNode = node[0]
          targetReached = True
          break
      ###if we didnt arrive to the end point will expand the nodes and check his actions
      for action in problem.getSuccessors(node[0]):

          if action[0] not in explored.keys() and action[0] not in OrderedPath.keys():  # check if we already did the action
              OrderedPath[action[0]] = node[0]
              frontier.push(action)

  while (targetNode in OrderedPath.keys()):  # if we got a solution lets find the path to it

      lastNode = OrderedPath[targetNode]
      solution.insert(0, explored[targetNode])
      targetNode = lastNode  # go to the node before
  return solution
  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  node = problem.getStartState()  # the start of the map
  path_cost={} #a set that will hold all the path costs
  solution = []
  frontier = util.PriorityQueue() #thanks util.py
  frontier.push((node, 'Spawn', 0),0)  # pushing the spawn location to the frontier with zero cost
  explored = {}  # an empty set
  explored[node] = 'Spawn'
  OrderedPath = {}  # so we can trace back and show the order of the scan
  path_cost[node]=0

  if problem.isGoalState(node):  # checking if we spawned at the target
      return solution

  targetReached = False

  while (frontier and not targetReached):  # while we are not in the end and we still have options to scan

      node = frontier.pop()
      explored[node[0]] = node[1]  # put the node and his direction in the explored set

      if problem.isGoalState(node[0]):
          targetNode = node[0]
          targetReached = True
          break
      ###if we didnt arrive to the end point will expand the nodes and check his actions
      for action in problem.getSuccessors(node[0]):

          if action[0] not in explored.keys():  # check if we already did the action

              current_cost=node[2] + action[2] #summing up the  total cost of the current path
              if action[0]  in path_cost.keys():
                    if path_cost[action[0]]<=current_cost: # if the current path cost is higher than the last path dont do anything!
                        continue
              ### if we found a shorther path we will change the priority of the queue ! like fig 3.14 PG 84
              frontier.push((action[0], action[1], current_cost), current_cost) ## pushing the node correctly according to the new cost rule
              path_cost[action[0]] = current_cost
              OrderedPath[action[0]] = node[0]


  while (targetNode in OrderedPath.keys()):  # if we got a solution lets find the path to it

      lastNode = OrderedPath[targetNode]
      solution.insert(0, explored[targetNode])
      targetNode = lastNode  # go to the node before
  return solution


  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"

  node = problem.getStartState()  # the start of the map
  path_cost = {}  # a set that will hold all the path costs
  solution = []
  frontier = util.PriorityQueue()  # thanks util.py
  frontier.push((node, 'Spawn', 0), 0)  # pushing the spawn location to the frontier with zero cost
  explored = {}  # an empty set
  explored[node] = 'Spawn'
  OrderedPath = {}  # so we can trace back and show the order of the scan
  path_cost[node] = 0

  if problem.isGoalState(node):  # checking if we spawned at the target
      return solution

  targetReached = False

  while (frontier and not targetReached):  # while we are not in the end and we still have options to scan

      node = frontier.pop()
      explored[node[0]] = node[1]  # put the node and his direction in the explored set

      if problem.isGoalState(node[0]):
          targetNode = node[0]
          targetReached = True
          break
      ###if we didnt arrive to the end point will expand the nodes and check his actions
      for action in problem.getSuccessors(node[0]):

          if action[0] not in explored.keys():  # check if we already did the action

              current_cost = node[2] + action[2] + heuristic(action[0],problem) # summing up the  total cost of the current path and addint the Heursistic function!
              if action[0] in path_cost.keys():
                  if path_cost[action[0]] <= current_cost:  # if the current path cost is higher than the last path dont do anything!
                      continue
              ### if we found a shorther path we will change the priority of the queue ! like fig 3.14 PG 84
              frontier.push((action[0], action[1], node[2]+ action[2]),current_cost)  ## pushing the node correctly according to the new cost rule for A*
              path_cost[action[0]] = current_cost
              OrderedPath[action[0]] = node[0]

  while (targetNode in OrderedPath.keys()):  # if we got a solution lets find the path to it

      lastNode = OrderedPath[targetNode]
      solution.insert(0, explored[targetNode])
      targetNode = lastNode  # go to the node before
  return solution

  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch