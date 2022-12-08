import random
import numpy as np

from utilities.pr_barrier_certs import *
from utilities.controllers import *
from rps.utilities.barrier_certificates import *

from utilities import util
from utilities.Telemetry import Telemetry

""" CREATE FUNCTIONS FOR NORMAL MULTI CONTROLLER"""
# TODO: Optimize by using util.py functions instead of these
def get_robots():

    bodies = []
    left_motors = []
    right_motors = []
    targets = []

    count = 0
    loop = True
    while loop:
        try:
            bodies.append(sim.getObject('/Khepera_IV[%s]' % count))
            left_motors.append(sim.getObject('/Khepera_IV[%s]/K4_Left_Motor' % count))
            right_motors.append(sim.getObject('/Khepera_IV[%s]/K4_Right_Motor' % count))
            targets.append(sim.getObject('/Target_%s' % count))
            stop_robot(left_motors[count], right_motors[count])
            count += 1
        except:
            loop = False
    # print(bodies, left_motors, right_motors, targets, count)
    # 15 32 35 45
    # 46 63 66 76
    return bodies, left_motors, right_motors, targets, count


def ultrasonic_status(is_disabled, robot_index):
    ultrasonic_sensor = [-1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObject('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i)), 1)
    else:
        for i in range(1, 5):
            ultrasonic_sensor[i] = sim.getObject('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i))

    return ultrasonic_sensor


def infrared_status(is_disabled, robot_index):
    infrared_sensor = [-1, -1, -1, -1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObjectHandle('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i)), 1)
    else:
        for i in range(1, 5):
            infrared_sensor[i] = sim.getObjectHandle('/Khepera_IV[%s]/K4_Infrared_%s' % (robot_index, i))

    return infrared_sensor


def get_robot_position(body):
    pos = sim.getObjectPosition(body, -1)
    ori_body = sim.getObjectOrientation(body, -1)
    theta = ori_body[2]
    xc = pos[0]
    yc = pos[1]

    return xc, yc, theta


def get_target_position(target):
    tar = sim.getObjectPosition(target, -1)
    xp = tar[0]
    yp = tar[1]

    return xp, yp


def control(xp, yp, xc, yc, theta, index):
    global Kp, flag, Vmax, Wmax

    d = math.sqrt(((xp - xc) ** 2) + ((yp - yc) ** 2))
    alpha = math.atan2(yp - yc, xp - xc)
    Oc = alpha - theta

    if d >= 0.05:
        w = Wmax * math.sin(Oc)
        v = Kp[index] * d
        if v > Vmax:
            v = Vmax

        flag[index] = 0
    else:
        v = 0
        w = 0
        flag[index] = 1

    return v, w


def update_velocities(left_motor, right_motor, v, w):
    Vr = (2 * v + w * 0.1) / 2
    Vl = (2 * v - w * 0.1) / 2

    sim.setJointTargetVelocity(left_motor, 47.5930 * Vl)
    sim.setJointTargetVelocity(right_motor, 47.5930 * Vr)


def stop_robot(left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


def start_robot(left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 1)
    sim.setJointTargetVelocity(right_motor, 1)


def is_all_stopped(arr):
    for i in range(len(arr)):
        if not arr[i]:
            return False

    return True

""" INITIALIZE EVERYTHING """
# Start Sim and get all agents (robots)
sim, client = util.start_sim()
agents, targets, n = util.init_robots(sim)


# Initialize velocity vector for agents. Each agent expects a 2x1 velocity
#   vector containing the linear and angular velocity, respectively.
targets = np.array([util.get_target_position(sim, target) for target in targets])
x_goal = targets.T

# PrSBC Parameters
safety_radius = 0.20
confidence_level = 0.90

uni_barrier_cert = create_pr_unicycle_barrier_certificate_cent(safety_radius=safety_radius, confidence_level=confidence_level)
position_uni_clf_controller = create_clf_unicycle_pose_controller()

loop = True
stopped = np.zeros(len(agents))
# error variables
v_rand_span = 0.005 * np.ones((2, n)) # setting up velocity error range for each robot

# uniform distribution of noise
x_rand_span_x = 0.02 * np.random.randint(3, 4, (1, n)) # setting up position error range for each robot,
x_rand_span_y = 0.02 * np.random.randint(1, 4, (1, n)) # rand_span serves as the upper bound of uncertainty for each of the robot

x_rand_span_xy = np.concatenate((x_rand_span_x, x_rand_span_y))

pos_error = np.zeros((2, n))

# graphing
test_name = input('Enter name of Map: ')
t_PrSBC = Telemetry(len(agents), test_name, 'PrSBC', do_uBox=True)
t_SBC = Telemetry(len(agents), test_name, 'SBC', do_uBox=True)
t_NS = Telemetry(len(agents), test_name, 'Normal Controller', do_uBox=False)

"""PRSBC"""
print('STARTING PRSBC')
while loop:

    # ALGORITHM
    positions = np.array([agent.get_position(sim) for agent in agents])

    # reshape poses so that its 2 x n
    x = positions.T.copy()


    # Add fake noise over ground-truth poses (will get removed for real-world uses)
    x[:2, :] += pos_error
    pos_error = x_rand_span_xy * ((np.random.rand(2, n) - 0.5) * 2.0)
    # x[:2, :] += .1 * random.random()

    # get distance to gaol for all robots
    d = np.sqrt((x_goal[0] - x[0]) ** 2 + (x_goal[1] - x[1]) ** 2)

    # stop if distance threshold is met
    if (d < .05).all() or (stopped == 1).all():
        util.stop_all(sim, agents)
        loop = False
        continue

    # Use a position controller to drive to the goal position
    dxu = position_uni_clf_controller(x, x_goal)

    # Use the barrier certificates to make sure that the agents don't collide
    dxu = uni_barrier_cert(dxu, x, x_rand_span_xy, v_rand_span)

    for i in range(len(agents)):
        if round(d[i], 2) < .05 or stopped[i] == 1:
            dxu[0, i] = 0
            dxu[1, i] = 0
            stopped[i] = 1

    util.set_velocities(sim, agents, dxu)

    # graphing
    t_PrSBC.update(x, positions)

sim.stopSimulation()

util.wait_for_next()

"""SBC"""
loop = True
stopped = np.zeros(len(agents))
pos_error = np.zeros((2, n))
sim.startSimulation()
agents, targets, n = util.init_robots(sim)
targets = np.array([util.get_target_position(sim, target) for target in targets])
x_goal = targets.T
uni_barrier_cert = create_unicycle_barrier_certificate(safety_radius=safety_radius)

print('STARTING SBC')
while loop:

    # ALGORITHM
    positions = np.array([agent.get_position(sim) for agent in agents])

    # reshape positions so that its 2 x n
    x = positions.T.copy()

    # add noise
    x[:2, :] += pos_error
    pos_error = x_rand_span_xy * ((np.random.rand(2, n) - 0.5) * 2.0)
    # x[:2, :] += .1 * random.random()

    # get distance to gaol for all robots
    d = np.sqrt((x_goal[0] - x[0]) ** 2 + (x_goal[1] - x[1]) ** 2)

    # stop if distance threshold is met
    if (d < .05).all() or (stopped == 1).all():
        util.stop_all(sim, agents)
        loop = False
        continue

    # Use a position controller to drive to the goal position
    dxu = position_uni_clf_controller(x, x_goal)


    for i in range(len(agents)):
        if round(d[i], 2) < .05 or stopped[i] == 1:
            dxu[0, i] = 0
            dxu[1, i] = 0
            stopped[i] = 1

    # Use the barrier certificates to make sure that the agents don't collide
    dxu = uni_barrier_cert(dxu, x)

    util.set_velocities(sim, agents, dxu)

    t_SBC.update(x, positions)

sim.stopSimulation()

util.wait_for_next()


"""Normal Controller
sim.startSimulation()
Kp = []
flag = []
Vmax = 0.08
Wmax = math.pi / 2

# init robot
agents, _, n = util.init_robots(sim)
bodies, left_motors, right_motors, targets, count = get_robots()

# set sizes
zeros = np.zeros(count)
Kp = np.ones(count)
flag = zeros

stopped = np.zeros(count, dtype=bool)

for i in range(count):
    ultrasonic_status(True, i)
    infrared_status(True, i)

loop = True
print('STARTING NORMAL CONTROLLER')
while loop:
    for i in range(count):
        print('Robot %s' % i)
        if not stopped[i]:
            x_curr, y_curr, theta = get_robot_position(bodies[i])
            x_target, y_target = get_target_position(targets[i])

            v, w = control(x_target, y_target, x_curr, y_curr, theta, i)
            update_velocities(left_motors[i], right_motors[i], v, w)

            distance = math.sqrt((x_target - x_curr) ** 2 + (y_target - y_curr) ** 2)
            # print('v: %s\nw: %s\nDistance: %s\n' % (v, w, distance))

        # if robot stops moving or distance meets threshold

            if distance < 0.04:
                print('Robot %s has been stopped' % i)
                stop_robot(left_motors[i], right_motors[i])
                stopped[i] = True
        # print('-----------------------------------')
    positions = np.array([agent.get_position(sim) for agent in agents])
    t_NS.update(positions.T, positions)

    if is_all_stopped(stopped):
        loop = False

input('Press any key to continue')
sim.stopSimulation()
"""
""" TELEMETRY """

t_PrSBC.create_graph(.33, False, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_PrSBC.create_graph(.66, False, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_PrSBC.create_graph(error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_PrSBC.create_graph(min_dist=True)
t_PrSBC.create_graph(minest_dist=True, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)

t_SBC.create_graph(.33, False, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_SBC.create_graph(.66, False, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_SBC.create_graph(error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
t_SBC.create_graph(min_dist=True)
t_SBC.create_graph(minest_dist=True, error_bound_x=x_rand_span_x, error_bound_y=x_rand_span_y)
"""
t_NS.create_graph(.33, False)
t_NS.create_graph(.66, False)
t_NS.create_graph()

# t_NS.create_graph(min_dist=True)
# t_NS.create_graph(minest_dist=True)
"""

""" COMBO TELEMETRY """
t_PrSBC.create_graph(min_dist=True, t_obj=[t_SBC])