import random
import math
from environment import Agent, Environment
from simulator import Simulator
import sys
from searchUtils import searchUtils

class SearchAgent(Agent):
    """ An agent that drives in the Smartcab world.
        This is the object you will be modifying. """ 

    def __init__(self, env,location=None):
        super(SearchAgent, self).__init__(env)     # Set the agent in the evironment 
        self.valid_actions = self.env.valid_actions  # The set of valid actions
        self.action_sequence=[]
        self.searchutil = searchUtils(env)

    def choose_action(self):
        """ The choose_action function is called when the agent is asked to choose
            which action to take next"""

        # Set the agent state and default action
        action=None
        if len(self.action_sequence) >=1:
            action = self.action_sequence[0] 
        if len(self.action_sequence) >=2:
            self.action_sequence=self.action_sequence[1:]
        else:
            self.action_sequence=[]
        return action

    def drive(self, goalstates, inputs):
        """Write your algorithm for self driving car"""
        # extract start and goal states from environment
        path_list = []
        destination_reached = []
        action_order = []
        path_lengths = []
        start = self.state

        # for all goal states do
        #   goalReached,path= A Star(start, goal)
        for goal in goalstates:
            goalReached, path = self.AStar(start["location"], goal, inputs)
            destination_reached.append(goalReached)
            path_list.append(path)
            path_lengths.append(len(path))

        # Find best path from all paths received [1 path received for 1 goal]
        if True in destination_reached:
            # Best path, would the shortest path in case of goal is reachable 
            best_path = [path_list[i] for i in range(len(path_list)) if (destination_reached[i] == True) and (len(path_list[i]) == min(path_lengths))]
            action_order.extend(best_path)
        else:
            # otherwise it would be the longest path, how far traveled before being blocked 
            longest_path = [path_list[i] for i in range(len(path_list)) if (len(path_list[i]) == max(path_lengths))]
            action_order.extend(longest_path)

        # Compute action sequence for best path
        movements = {
            (0, 3): "forward-3x", 
            (0, 2): "forward-2x", 
            (0, 1): "forward", 
            (-1, 1): "left", 
            (1, 1): "right", 
            (0, 0): None}
        try:
            action_sequence = [movements[(action_order[0][i+1][0] - action_order[0][i][0], action_order[0][i+1][1] - action_order[0][i][1])] for i in range(len(action_order[0])-1)]
        except:
            action_sequence = [None]
        # return action sequence
        return action_sequence

    def AStar(self, start, goal, state):
        # closedSet = []
        closedSet = []
        # openSet = []
        openSet = [start]
        # cameFrom = []
        cameFrom = {}
        # gScore[start] = 0
        gScore = {start: 0}
        # fScore[start] = heuristic_cost_estimate(start, goal)
        fScore = {start: self.heuristic_cost_estimate(start, goal)}
        # current = []
        current = []

        # while openSet is not empty do
        while len(openSet) > 0:
            # current = the node in openSet having the lowest fScore value
            min_idx = list(fScore.values()).index(min(fScore.values()))
            current = list(fScore.keys())[min_idx]
            # if current = goal then
            if current == goal["location"]:
                # return Goal_Reached, reconstruct_path(cameFrom, current)
                return True, self.reconstruct_path(cameFrom, current)
            # remove current from openSet and fScore
            del fScore[current]
            openSet.remove(current)
            # add current to closedSet
            closedSet.append(current)

            # for all actions do
            for action in ["forward-3x", "forward-2x", "forward", "left", "right"]:
                # neighbor = applyAction(current, action)
                neighbor = self.applyAction(current, action, state, goal["location"])
                # if neighbor = current then
                if neighbor == current:
                    # path blocked, stop, evaluate next action
                    continue
                # tentative gScore = gScore[current] + Distance from start to neighbor
                tentative_gScore = gScore[current] + 1
                # if neighbor not in openSet then
                if neighbor not in openSet:
                    # Add neighbor in openSet
                    openSet.append(neighbor)
                # else if tentative gScore ≥ gScore[neighbor] then
                elif tentative_gScore >= gScore[neighbor]:
                    # stop, evaluate next action
                    continue
                # cameFrom[neighbor] = current
                cameFrom[neighbor] = current
                # gScore[neighbor] = tentative gScore
                gScore[neighbor] = tentative_gScore
                # fScore[neighbor] = gScore[neighbor] + heuristic_cost_estimate(neighbor, goal)
                fScore[neighbor] = gScore[neighbor] + self.heuristic_cost_estimate(neighbor, goal)
        
        # if current = goal then
        if current == goal["location"]:
            # return Goal Reached, reconstruct path(cameFrom, current)
            return True, self.reconstruct_path(cameFrom, current)
        # else
        else:
            # return Goal Not Reachable, reconstruct_path(cameFrom, current)
            return False, self.reconstruct_path(cameFrom, current)

    def heuristic_cost_estimate(self, start, goal):
        # return [how far ahead is goal state]/2
        result = abs(goal["location"][1] - start[1]) + abs(goal["location"][0] - start[0]) / 2
        return result

    def applyAction(self, current, action, state, goal):
        # define movement paths
        movements = {
            "forward-3x": [(0, 1), (0, 2), (0, 3)], 
            "forward-2x": [(0, 1), (0, 2)], 
            "forward": [(0, 1)], 
            "left": [(-1, 0), (-1, 1)], 
            "right": [(1, 0), (1, 1)], 
            None: [(0, 0)]}
        
        # defining row and col
        target_row = current[0]
        target_col = current[1]
        # define check if movement is valid
        valid_move = True

        # for each movement in the path
        for movement in movements[action]:
            # if movement is outside of the grid
            if (target_row + movement[0] >= len(state)) or \
                (target_row + movement[0] < 0) or \
                (target_col + movement[1]) >= len(state[0]):
                # return current location
                return current
            
            # if movement has a car in the way
            if state[target_row + movement[0]][target_col + movement[1]] == 1:
                # declare check as False
                valid_move = False
                # stop all subsequent movements
                break
        
        # if movement leads to a location where there will not be a path to the goal
        if abs(target_row + movements[action][-1][0] - goal[0]) > abs(goal[1] - (target_col + movements[action][-1][1])):
            # return current location
            return current

        # if movement is valid
        if valid_move:
            # add movement to the current row and col
            target_row += movements[action][-1][0]
            target_col += movements[action][-1][1]
        # return endstate
        return (target_row, target_col)
    

    def reconstruct_path(self, cameFrom, current):
        # total_path = []
        total_path = []
        # while current in cameFrom.keys do
        while current in cameFrom.keys():
            # total_path.append(current)
            total_path.append(current)
            # current = cameFrom[current] total path.append(current) return total_path
            current = cameFrom[current] 
        # add the last node into list
        total_path.append(current)
        # flip list
        total_path = total_path[::-1]
        # return total_path
        return total_path

    def update(self):
        """ The update function is called when a time step is completed in the 
            environment for a given trial. This function will build the agent
            state, choose an action, receive a reward, and learn if enabled. """
        startstate = self.state
        goalstates =self.env.getGoalStates()
        inputs = self.env.sense(self)
        self.action_sequence = self.drive(goalstates,inputs)
        action = self.choose_action()  # Choose an action
        self.state = self.env.act(self,action)        
        return
        

def run(filename):
    """ Driving function for running the simulation. 
        Press ESC to close the simulation, or [SPACE] to pause the simulation. """

    env = Environment(config_file=filename,fixmovement=False)
    
    agent = env.create_agent(SearchAgent)
    env.set_primary_agent(agent)
    
    ##############
    # Create the simulation
    # Flags:
    #   update_delay - continuous time (in seconds) between actions, default is 2.0 seconds
    #   display      - set to False to disable the GUI if PyGame is enabled
    sim = Simulator(env, update_delay=2)
    
    ##############
    # Run the simulator
    ##############
    sim.run()


if __name__ == '__main__':
    run(sys.argv[1])
