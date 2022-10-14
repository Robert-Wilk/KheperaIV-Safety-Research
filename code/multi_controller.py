from zmqRemoteApi import RemoteAPIClient
import math
import numpy as np

"""
Controls multiple Khepera IV robots in Coppelia Sim

The program will grab all available robots in a scene.
The robot will move to their target destination.
Once all targets are reached, the program will
wait for user input to exit the simulation.
"""

# sim setup
client = RemoteAPIClient()
sim = client.getObject('sim')

# global variables
Kp = []
flag = []
Vmax = 0.08
Wmax = math.pi / 2


def main():
    global Kp, flag

    # init robot
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
    while loop:
        for i in range(count):
            print('Robot %s' % i)
            if not stopped[i]:
                x_curr, y_curr, theta = get_robot_position(bodies[i])
                x_target, y_target = get_target_position(targets[i])

                v, w = control(x_target, y_target, x_curr, y_curr, theta, i)
                update_velocities(left_motors[i], right_motors[i], v, w)

                distance = math.sqrt((x_target - x_curr) ** 2 + (y_target - y_curr) ** 2)
                print('v: %s\nw: %s\nDistance: %s\n' % (v, w, distance))

            # if robot stops moving or distance meets threshold

                if distance < 0.04:
                    print('Robot %s has been stopped' % i)
                    stop_robot(left_motors[i], right_motors[i])
                    stopped[i] = True
            print('-----------------------------------')

        if is_all_stopped(stopped):
            loop = False

    input('Press any key to continue')
    sim.stopSimulation()


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


if __name__ == '__main__':
    sim.startSimulation()
    main()
