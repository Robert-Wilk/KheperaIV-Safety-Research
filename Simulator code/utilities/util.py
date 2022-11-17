from zmqRemoteApi import RemoteAPIClient
import math
import numpy as np

"""
A collection of functions to interface simulation and Khepera IV Robots

Takes and OOP approach to manage the robots.
"""


# TODO: create/edit documentation for all function
class Robot:
    def __init__(self, body, left_motor, right_motor):
        self.body = body
        self.left_motor = left_motor
        self.right_motor = right_motor

        self.Kp = 1
        self.flag = 0
        self.Vmax = 0.08
        self.Wmax = math.pi / 2

    def get_flag(self):
        return self.flag

    def get_position(self, sim):
        pos = sim.getObjectPosition(self.body, -1)
        ori_body = sim.getObjectOrientation(self.body, -1)
        theta = ori_body[2]
        xc = pos[0]
        yc = pos[1]

        return xc, yc, theta

    def control(self, xp, yp, xc, yc, theta):
        # TODO add theta and position to robot class
        d = math.sqrt(((xp - xc) ** 2) + ((yp - yc) ** 2))
        alpha = math.atan2(yp - yc, xp - xc)
        Oc = alpha - theta

        if d >= 0.05:
            w = self.Wmax * math.sin(Oc)
            v = self.Kp * d
            if v > self.Vmax:
                v = self.Vmax

            self.flag = 0
        else:
            v = 0
            w = 0
            self.flag = 1

        return v, w

    def update_velocity(self, sim, v, w):
        Vr = (2 * v + w * 0.1) / 2
        Vl = (2 * v - w * 0.1) / 2

        sim.setJointTargetVelocity(self.left_motor, 47.5930 * Vl)
        sim.setJointTargetVelocity(self.right_motor, 47.5930 * Vr)

    def stop(self, sim):
        sim.setJointTargetVelocity(self.left_motor, 0)
        sim.setJointTargetVelocity(self.right_motor, 0)

    def start(self, sim):
        sim.setJointTargetVelocity(self.left_motor, 1)
        sim.setJointTargetVelocity(self.right_motor, 1)


def start_sim():
    """
    Starts Simulation for Coppelia Sim

    :return:
    """
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    sim.startSimulation()

    return sim, client


def init_robots(sim):
    robots = []
    targets = []

    count = 0
    loop = True
    while loop:
        try:
            body = sim.getObject('/Khepera_IV[%s]' % count)
            left_motor = sim.getObject('/Khepera_IV[%s]/K4_Left_Motor' % count)
            right_motor = sim.getObject('/Khepera_IV[%s]/K4_Right_Motor' % count)
            target = sim.getObject('/Target_%s' % count)

            agent = Robot(body, left_motor, right_motor)
            targets.append(target)

            robots.append(agent)

            agent.stop(sim)

            count += 1
        except:
            loop = False

    for i in range(count):
        ultrasonic_status(sim, True, i)
        infrared_status(sim, True, i)

    return robots, targets, count


def is_all_stopped(arr):
    for i in range(len(arr)):
        if not arr[i]:
            return False

    return True


def ultrasonic_status(sim, is_disabled, robot_index):
    ultrasonic_sensor = [-1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObject('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i)), 1)
    else:
        for i in range(1, 5):
            ultrasonic_sensor[i] = sim.getObject('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i))

    return ultrasonic_sensor


def infrared_status(sim, is_disabled, robot_index):
    infrared_sensor = [-1, -1, -1, -1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObjectHandle('/Khepera_IV[%s]/K4_Ultrasonic_%s' % (robot_index, i)), 1)
    else:
        for i in range(1, 5):
            infrared_sensor[i] = sim.getObjectHandle('/Khepera_IV[%s]/K4_Infrared_%s' % (robot_index, i))

    return infrared_sensor


def get_target_position(sim, target):
    tar = sim.getObjectPosition(target, -1)
    xp = tar[0]
    yp = tar[1]

    return xp, yp


def set_velocities(sim, robots, vs):
    """ Sets the velocities of the current agents.
        Parameters
        ----------
        ids : array of int
            Identities of agents whose velocities to set.
        vs : array of ints
            Velocities to set.
    """
    n = vs.shape[1]

    for i in range(0, n):
        # index 0 is linear values (0v, 1w, 2w, ...)
        # index 1 is angular values (0w, 1w, 2w, ...)

        # check linear and angular velocity
        if np.absolute(vs[0, i]) > robots[i].Vmax:
            vs[0, i] = robots[i].Vmax * np.sign(vs[0, i])

        if np.absolute(vs[1, i]) > robots[i].Wmax:
            vs[1, i] = robots[i].Wmax * np.sign(vs[1, i])

        robots[i].update_velocity(sim, vs[0, i], vs[1, i])

    # Change the state of agents within the ids array


# TODO: get it to actually stop the robots
def stop_all(sim, robots):
    for r in robots:
        r.stop(sim)


def wait_for_next():
    loop = True
    while loop:
        done = input('Enter 1 for done:')
        try:
            if int(done) == 1:
                loop = False
        except:
            loop = True
