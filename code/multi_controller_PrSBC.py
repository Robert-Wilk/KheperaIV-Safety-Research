from rps.utilities.barrier_certificates import *
from rps.utilities.controllers import *

from utilities import util

import numpy as np


"""
Implements PrSBC on multiple Khepera IV robots in Coppelia Sim

The program will grab all available robots in a scene.
The robot will move to their target destination.
Once all targets are reached, the program will
wait for user input to exit the simulation. The program
will also make sure that robots do not collide.

PSEUDOCODE:
1. Start simulation
2. Get all agents
3. Initialize velocity vector for agents. Each agent expects a 2x1 velocity
   vector containing the linear and angular velocity, respectively.
4. Iterate until all agents reach goal:
5.   Retrieve the most recent poses of the robots
6.   Control agents
7.   Check collisions of agents
8.   Calculate and update agents' velocities
"""

# TODO: modify to create PrSBC from looking at Dr.Luo's Github
# Start simulation
sim, client = util.start_sim()

# Get all agents (which means to set n variable to the number of agents)
agents, targets, n = util.init_robots(sim)


# Initialize velocity vector for agents. Each agent expects a 2x1 velocity
#   vector containing the linear and angular velocity, respectively.
dx = np.zeros((2, n))
targets = np.array([util.get_target_position(sim, target) for target in targets])
x_goal = targets.T

# Default Barrier Parameters
safety_radius = 0.17

# We're working in single-integrator dynamics, and we don't want the robots
# to collide.  Thus, we're going to use barrier certificates
si_barrier_cert = create_single_integrator_barrier_certificate()

# Create single integrator position controller
si_position_controller = create_si_position_controller()

# Create SI to UNI dynamics tranformation
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

# These variables are so we can tell when the robots should switch positions
# on the circle.
flag = 0

# iterate for the previously specified number of iterations.
stopped = np.zeros(n, dtype=bool)

loop = True

while loop:

    # ALGORITHM
    positions = np.array([agent.get_position(sim) for agent in agents])
    # Nominal controller, go2goal

    # reshape positions so that its 2 x n
    x = positions.T

    x_si = uni_to_si_states(x)

    # Initialize a velocities variable
    si_velocities = np.zeros((2, n))

    if (np.linalg.norm(x_goal - x_si) < 0.05):
        flag = 1

    # These if statements are changing what goal to go towards
    if flag != 0:
        util.stop_all(sim, agents)
        loop = False
        continue

    # Use a position controller to drive to the goal position
    dxi = si_position_controller(x_si, x_goal)

    # Use the barrier certificates to make sure that the agents don't collide
    dxi = si_barrier_cert(dxi, x_si)

    # Use the second single-integrator-to-unicycle mapping to map to unicycle
    # dynamics
    dxu = si_to_uni_dyn(dxi, x)

    util.set_velocities(sim, agents, dxu)

input('Press any key to continue')
sim.stopSimulation()
