from zmqRemoteApi import RemoteAPIClient
import math
import numpy as np
from utilities.controllers import create_clf_unicycle_pose_controller

position_uni_clf_controller = create_clf_unicycle_pose_controller()

"""
Controls a single Khepera IV robot in Coppelia Sim

The user will be asked to specify a robot in the scene.
After a robot is specified, the robot will move to the 
target destination. Once target is reached, the program will
wait for user input to exit the simulation.
"""

# TODO: stop other robots from spinning when scene is first launched

# sim setup
client = RemoteAPIClient()
sim = client.getObject('sim')

# global variables
Kp = 1
flag = 0
d = 0
v = 0
w = 0
Vmax = 0.0
Wmax = 0.0


def main():
    global Kp, flag, d, v, w

    robot = int(input('Enter robot to move:'))
    # init robot
    body, left_motor, right_motor, target = set_robot(robot)
    ultrasonic_status(True)
    infrared_status(True)

    loop = True
    while loop:
        x_curr, y_curr, theta = get_robot_position(body)
        x_target, y_target = get_target_position(target)
        x_pose = np.array([[x_curr], [y_curr], [theta]])
        x_goal = np.array([[x_target], [y_target]])

        # flag, v, w = control(x_target, y_target, x_curr, y_curr, theta)
        dxu = position_uni_clf_controller(x_pose, x_goal)
        update_velocities(dxu[0,0], dxu[1,0], left_motor, right_motor)

        distance = math.sqrt((x_target - x_curr) ** 2 + (y_target - y_curr) ** 2)
        # print('v: %s\nw: %s\nDistance: %s\n' % (v, w, distance))

        # if robot stops moving or distance meets threshold
        if (dxu[0,0] == 0 and dxu[1,0] == 0) or distance < 0.04:
            stop_robot(left_motor, right_motor)
            loop = False

        # TODO: setup graphing

    input('Press any key to continue')
    sim.stopSimulation()


def set_robot(robot):
    global Vmax, Wmax

    Vmax = 0.08
    Wmax = math.pi / 2

    body = sim.getObjectHandle('/Khepera_IV[%s]' % robot)  # might have to include /
    left = sim.getObjectHandle('/Khepera_IV[%s]/K4_Left_Motor' % robot)
    right = sim.getObjectHandle('/Khepera_IV[%s]/K4_Right_Motor' % robot)

    target = sim.getObjectHandle('/Target_%s' % robot)
    sim.setObjectParent(target, -1, True)

    # print(body, left, right, target)
    # 15 32 35 45
    # 46 63 66 76
    return body, left, right, target


def ultrasonic_status(is_disabled):
    ultrasonic_sensor = [-1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObjectHandle('K4_Ultrasonic_%s' % i), 1)
    else:
        for i in range(1, 5):
            ultrasonic_sensor[i] = sim.getObjectHandle('K4_Ultrasonic_%s' % i)

    return ultrasonic_sensor


def infrared_status(is_disabled):
    infrared_sensor = [-1, -1, -1, -1, -1, -1, -1, -1]

    if is_disabled:
        for i in range(1, 5):
            sim.setExplicitHandling(sim.getObjectHandle('K4_Ultrasonic_%s' % i), 1)
    else:
        for i in range(1, 5):
            infrared_sensor[i] = sim.getObjectHandle('K4_Infrared_%s' % i)

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


def update_velocities(v, w, left_motor, right_motor):
    Vr = (2 * v + w * 0.1) / 2
    Vl = (2 * v - w * 0.1) / 2

    sim.setJointTargetVelocity(left_motor, 47.5930 * Vl)
    sim.setJointTargetVelocity(right_motor, 47.5930 * Vr)


def stop_robot(left_motor, right_motor):
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)


if __name__ == '__main__':
    sim.startSimulation()
    main()
