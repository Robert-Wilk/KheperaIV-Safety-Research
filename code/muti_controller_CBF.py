from rps.utilities.barrier_certificates import *
from utilities.controllers import *

from utilities import util

import numpy as np


"""
Implements CBF on multiple Khepera IV robots in Coppelia Sim

The program will grab all available robots in a scene.
The robot will move to their target destination.
Once all targets are reached, the program will
wait for user input to exit the simulation. The program
will also make sure that robots do not collide.

PSEUDOCODE:
1. Start simulation
2. Get all agents
3. Initialize Robotarium helper functions
4. Iterate until all agents reach goal:
5.   Retrieve the most recent poses of the robots
6.   Control agents
7.   Check collisions of agents
8.   Calculate and update agents' velocities
"""

# Future tunings: understand gamma value and how that affects pose controller
# Future tunings: understand Quadradtic programing and how that impacts the controller
# TODO: Work on improving crashing in 8 robot scene

# Start Sim and get all agents (robots)
sim, client = util.start_sim()
agents, targets, n = util.init_robots(sim)


# Initialize velocity vector for agents. Each agent expects a 2x1 velocity
#   vector containing the linear and angular velocity, respectively.
targets = np.array([util.get_target_position(sim, target) for target in targets])
x_goal = targets.T

# Barrier Parameters
safety_radius = 0.20

# We're working in unicycle dynamics, and we don't want the robots
# to collide.  Thus, we're going to use barrier certificates
# TODO: play around with the unicycle barrier certificate functions robotarium offers
uni_barrier_cert = create_unicycle_barrier_certificate(safety_radius=safety_radius)

# Create unicycle pose controller
# Decided to use pose-based over position-based because the postion-based controller creates zig-zag motion
# TODO: see if we can move from using custom version of position_uni_clf_controller
position_uni_clf_controller = create_clf_unicycle_pose_controller()

loop = True

while loop:

    # ALGORITHM
    positions = np.array([agent.get_position(sim) for agent in agents])

    # reshape positions so that its 2 x n
    x = positions.T

    # Initialize a velocities variable
    si_velocities = np.zeros((2, n))

    # get distance to gaol for all robots
    d = np.sqrt((x_goal[0] - x[0]) ** 2 + (x_goal[1] - x[1]) ** 2)

    # stop if distance threshold is met
    if (d < .05).all():
        util.stop_all(sim, agents)
        loop = False
        continue

    # Use a position controller to drive to the goal position
    dxu = position_uni_clf_controller(x, x_goal)

    # Use the barrier certificates to make sure that the agents don't collide
    dxu = uni_barrier_cert(dxu, x)

    util.set_velocities(sim, agents, dxu)

input('Press any key to continue')
sim.stopSimulation()
