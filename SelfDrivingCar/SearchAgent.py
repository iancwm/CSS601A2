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

    def drive(self,goalstates,inputs):
        """Write your algorithm for self driving car"""
        act_sequence=[]
        return act_sequence        



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
