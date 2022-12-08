from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp, options
from cvxopt import matrix, sparse

from scipy.special import comb
from rps.utilities.transformations import *

# Disable output of CVXOPT
options['show_progress'] = False
# Change default options of CVXOPT for faster solving
options['reltol'] = 1e-2 # was e-2
options['feastol'] = 1e-2 # was e-4
options['maxiters'] = 50 # default is 100

def create_pr_unicycle_barrier_certificate_cent(barrier_gain=100, safety_radius=0.12, projection_distance=0.05, magnitude_limit=0.2, confidence_level=0.90):
    """
    MODIFIED VERSION OF create_unicycle_barrier_certificate FROM ROBOTARIUM

    Creates a unicycle Probability Safety barrier cetifcate to avoid collisions. Uses the diffeomorphism mapping
    and single integrator implementation. For optimization purposes, this function returns
    another function.

    barrier_gain: double (how fast the robots can approach each other)
    safety_radius: double (how far apart the robots should stay)
    projection_distance: double (how far ahead to place the bubble)

    -> function (the unicycle barrier certificate function)

    CREATED BY: Robert Wilk
    LAST MODIFIED: 10/19/2022
    """

    #Check user input types
    assert isinstance(barrier_gain, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(barrier_gain).__name__
    assert isinstance(safety_radius, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(safety_radius).__name__
    assert isinstance(projection_distance, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(magnitude_limit, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(magnitude_limit).__name__
    assert isinstance(confidence_level, float), "In the function create_pr_unicycle_barrier_certificate, the confidence level must be a float. Recieved type %r." % type(confidence_level).__name__

    #Check user input ranges/sizes
    assert barrier_gain > 0, "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m). Recieved %r." % safety_radius
    assert projection_distance > 0, "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be positive. Recieved %r." % projection_distance
    assert magnitude_limit > 0, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit
    assert confidence_level <= 1, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be less than 1. Recieved %r." % confidence_level
    assert confidence_level >= 0, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be positive (greater than 0). Recieved %r." % confidence_level

    si_barrier_cert = create_pr_si_barrier_certificate(gamma=barrier_gain, safety_radius=safety_radius+projection_distance, confidence_level=confidence_level)

    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping(projection_distance=projection_distance)

    uni_to_si_dyn = create_uni_to_si_dynamics(projection_distance=projection_distance)

    def f(dxu, x, XRandSpan=None, URandSpan=None):

        if URandSpan is None:
            URandSpan = np.zeros((2, x.shape[1]))
        if XRandSpan is None:
            XRandSpan = np.zeros((2, x.shape[1]))

        #Check user input types
        assert isinstance(dxu, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the unicycle robot velocity command (dxu) must be a numpy array. Recieved type %r." % type(dxu).__name__
        assert isinstance(x, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

        #Check user input ranges/sizes
        assert x.shape[0] == 3, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the unicycle robot states (x) must be 3 ([x;y;theta]). Recieved dimension %r." % x.shape[0]
        assert dxu.shape[0] == 2, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the robot unicycle velocity command (dxu) must be 2 ([v;w]). Recieved dimension %r." % dxu.shape[0]
        assert x.shape[1] == dxu.shape[1], "In the function created by the create_unicycle_barrier_certificate function, the number of robot states (x) must be equal to the number of robot unicycle velocity commands (dxu). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxu.shape[0], dxu.shape[1])


        x_si = uni_to_si_states(x)
        #Convert unicycle control command to single integrator one
        dxi = uni_to_si_dyn(dxu, x)
        #Apply single integrator barrier certificate
        dxi = si_barrier_cert(dxi, x_si, XRandSpan, URandSpan)
        #Return safe unicycle command
        return si_to_uni_dyn(dxi, x)

    return f


def create_pr_si_barrier_certificate(gamma=100, safety_radius=0.17, magnitude_limit=0.2, confidence_level=1,
                                     XRandSpan=None, URandSpan=None):
    """
    MODIFIED VERSION OF create_si_barrier_certificate FROM ROBOTARIUM

    Creates a barrier certificate for a single-integrator system.  This function
    returns another function for optimization reasons.

    gamma: double (controls how quickly agents can approach each other.  lower = slower)
    safety_radius: double (how far apart the agents will stay)
    magnitude_limit: how fast the robot can move linearly.

    -> function (the barrier certificate function)

    CREATED BY: Robert Wilk
    LAST MODIFIED: 10/19/2022
    """

    # Check user input types

    # gamma = barrier_gain
    if URandSpan is None:
        URandSpan = [0]
    if XRandSpan is None:
        XRandSpan = [0]

    assert isinstance(gamma, (int,
                                     float)), "In the function create_single_integrator_barrier_certificate, the barrier gain (gamma) must be an integer or float. Recieved type %r." % type(
        gamma).__name__
    assert isinstance(safety_radius, (int,
                                      float)), "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(
        safety_radius).__name__
    assert isinstance(magnitude_limit, (int,
                                        float)), "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(
        magnitude_limit).__name__

    # Check user input ranges/sizes
    assert gamma > 0, "In the function create_single_integrator_barrier_certificate, the barrier gain (gamma) must be positive. Recieved %r." % gamma
    assert safety_radius >= 0.12, "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m) plus the distance to the look ahead point used in the diffeomorphism if that is being used. Recieved %r." % safety_radius
    assert magnitude_limit > 0, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit


    def f(dxi, x, XRandSpan, URandSpan):
        # Check user input types
        assert isinstance(dxi,
                          np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(
            dxi).__name__
        assert isinstance(x,
                          np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(
            x).__name__

        # Check user input ranges/sizes
        assert x.shape[
                   0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % \
                            x.shape[0]
        assert dxi.shape[
                   0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % \
                            dxi.shape[0]
        assert x.shape[1] == dxi.shape[
            1], "In the function created by the create_single_integrator_barrier_certificate function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (
        x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])

        # Initialize some variables for computational savings
        N = dxi.shape[1]
        num_constraints = int(comb(N, 2))
        A = np.zeros((num_constraints, 2 * N))
        b = np.zeros(num_constraints)
        H = sparse(matrix(2 * np.identity(2 * N)))

        # TODO: check that these are going down the correct axis
        if len(XRandSpan) == 1:
            XRandSpan = np.zeros(2, N)
        if len(URandSpan) == 1:
            URandSpan = np.zeros(2, N)


        count = 0
        for i in range(N - 1):
            for j in range(i + 1, N):
                # each should be 1 value
                max_dvij_x = np.linalg.norm([URandSpan[0, i] + URandSpan[0, j]], 2)
                max_dvij_y = np.linalg.norm([URandSpan[1, i] + URandSpan[1, j]], 2)

                # each should be 1 value
                max_dxij_x = np.linalg.norm([x[0, i] - x[0, j]], 2) + np.linalg.norm([XRandSpan[0, i] + XRandSpan[0, j]], 2)
                max_dxij_y = np.linalg.norm([x[1, i] - x[1, j]], 2) + np.linalg.norm([XRandSpan[1, i] + XRandSpan[1, j]], 2)

                b2_x, b1_x, sigma = trap_cdf_inv(XRandSpan[0, i], XRandSpan[0, j], x[0, i] - x[0, j], confidence_level)
                b2_y, b1_y, sigma = trap_cdf_inv(XRandSpan[1, i], XRandSpan[1, j], x[1, i] - x[1, j], confidence_level)

                if (b2_x < 0 and b1_x > 0) or (b2_x > 0 and b1_x < 0):
                    # print('WARNING: distance between robots on x smaller than error bound!')
                    b_x = 0
                elif (b1_x < 0) and (b2_x < b1_x) or (b2_x < 0 and b2_x > b1_x):
                    b_x = b1_x
                elif (b2_x > 0 and b2_x < b1_x) or (b1_x > 0 and b2_x > b1_x):
                    b_x = b2_x
                else:
                    b_x = b1_x
                    # print('WARNING: no uncertainty or sigma = 0.5 on x')  # b1 = b2 or no uncertainty

                if (b2_y < 0 and b1_y > 0) or (b2_y > 0 and b1_y < 0):
                    # print('WARNING: distance between robots on y smaller than error bound!')
                    b_y = 0
                elif (b1_y < 0 and b2_y < b1_y) or (b2_y < 0 and b2_y > b1_y):
                    b_y = b1_y
                elif (b2_y > 0 and b2_y < b1_y) or (b1_y > 0 and b2_y > b1_y):
                    b_y = b2_y
                else:
                    b_y = b1_y
                    # print('WARNING: no uncertainty or sigma = 0.5 on y')

                A[count, (2 * i)] = -2 * b_x  # matlab original: A(count, (2*i-1):(2*i)) = -2*([b_x;b_y]);
                A[count, (2 * i + 1)] = -2 * b_y

                A[count, (2 * j)] = 2 * b_x  # matlab original: A(count, (2*j-1):(2*j)) =  2*([b_x;b_y])';
                A[count, (2 * j + 1)] = 2 * b_y

                h1 = np.linalg.norm([b_x, 0.0]) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm([max_dvij_x, 0]) * np.linalg.norm([max_dxij_x, 0]) / gamma
                h2 = np.linalg.norm([0, b_y]) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm([0, max_dvij_y]) * np.linalg.norm([0, max_dxij_y]) / gamma # h_y

                h = h1 + h2

                b[count] = gamma * h ** 3  # matlab original: b(count) = gamma*h^3
                count += 1

                '''
                OLD CODE

                # calculate error
                error = x[:, i] - x[:, j]
                h = (error[0] * error[0] + error[1] * error[1]) - np.power(safety_radius, 2)

                A[count, (2 * i, (2 * i + 1))] = -2 * error
                A[count, (2 * j, (2 * j + 1))] = 2 * error
                b[count] = barrier_gain * np.power(h, 3)

                count += 1
                '''

        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]

        f_mat = -2 * np.reshape(dxi, 2 * N, order='F')
        result = qp(H, matrix(f_mat), matrix(A), matrix(b))['x']

        return np.reshape(result, (2, -1), order='F')

    return f

def create_pr_unicycle_barrier_certificate_cent(barrier_gain=100, safety_radius=0.12, projection_distance=0.05, magnitude_limit=0.2, confidence_level=0.90):
    """
    MODIFIED VERSION OF create_unicycle_barrier_certificate FROM ROBOTARIUM

    Creates a unicycle Probability Safety barrier cetifcate to avoid collisions. Uses the diffeomorphism mapping
    and single integrator implementation. For optimization purposes, this function returns
    another function.

    barrier_gain: double (how fast the robots can approach each other)
    safety_radius: double (how far apart the robots should stay)
    projection_distance: double (how far ahead to place the bubble)

    -> function (the unicycle barrier certificate function)

    CENTRALIZED VERSION

    CREATED BY: Robert Wilk
    LAST MODIFIED: 10/19/2022
    """

    #Check user input types
    assert isinstance(barrier_gain, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be an integer or float. Recieved type %r." % type(barrier_gain).__name__
    assert isinstance(safety_radius, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(safety_radius).__name__
    assert isinstance(projection_distance, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(magnitude_limit, (int, float)), "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(magnitude_limit).__name__
    assert isinstance(confidence_level, float), "In the function create_pr_unicycle_barrier_certificate, the confidence level must be a float. Recieved type %r." % type(confidence_level).__name__

    #Check user input ranges/sizes
    assert barrier_gain > 0, "In the function create_pr_unicycle_barrier_certificate, the barrier gain (barrier_gain) must be positive. Recieved %r." % barrier_gain
    assert safety_radius >= 0.12, "In the function create_pr_unicycle_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m). Recieved %r." % safety_radius
    assert projection_distance > 0, "In the function create_pr_unicycle_barrier_certificate, the projected point distance for the diffeomorphism between sinlge integrator and unicycle (projection_distance) must be positive. Recieved %r." % projection_distance
    assert magnitude_limit > 0, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_pr_unicycle_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit
    assert confidence_level <= 1, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be less than 1. Recieved %r." % confidence_level
    assert confidence_level >= 0, "In the function create_pr_unicycle_barrier_certificate, the confidence level must be positive (greater than 0). Recieved %r." % confidence_level

    si_barrier_cert = create_pr_si_barrier_certificate(gamma=barrier_gain, safety_radius=safety_radius+projection_distance, confidence_level=confidence_level)

    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping(projection_distance=projection_distance)

    uni_to_si_dyn = create_uni_to_si_dynamics(projection_distance=projection_distance)

    def f(dxu, x, XRandSpan=None, URandSpan=None):

        if URandSpan is None:
            URandSpan = np.zeros((2, x.shape[1]))
        if XRandSpan is None:
            XRandSpan = np.zeros((2, x.shape[1]))

        #Check user input types
        assert isinstance(dxu, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the unicycle robot velocity command (dxu) must be a numpy array. Recieved type %r." % type(dxu).__name__
        assert isinstance(x, np.ndarray), "In the function created by the create_unicycle_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(x).__name__

        #Check user input ranges/sizes
        assert x.shape[0] == 3, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the unicycle robot states (x) must be 3 ([x;y;theta]). Recieved dimension %r." % x.shape[0]
        assert dxu.shape[0] == 2, "In the function created by the create_unicycle_barrier_certificate function, the dimension of the robot unicycle velocity command (dxu) must be 2 ([v;w]). Recieved dimension %r." % dxu.shape[0]
        assert x.shape[1] == dxu.shape[1], "In the function created by the create_unicycle_barrier_certificate function, the number of robot states (x) must be equal to the number of robot unicycle velocity commands (dxu). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (x.shape[0], x.shape[1], dxu.shape[0], dxu.shape[1])


        x_si = uni_to_si_states(x)
        #Convert unicycle control command to single integrator one
        dxi = uni_to_si_dyn(dxu, x)
        #Apply single integrator barrier certificate
        dxi = si_barrier_cert(dxi, x_si, XRandSpan, URandSpan)
        #Return safe unicycle command
        return si_to_uni_dyn(dxi, x)

    return f


def create_pr_si_barrier_certificate_d_cent(gamma=100, safety_radius=0.17, magnitude_limit=0.2, confidence_level=1,
                                     XRandSpan=None, URandSpan=None):
    """
    MODIFIED VERSION OF create_si_barrier_certificate FROM ROBOTARIUM

    Creates a barrier certificate for a single-integrator system.  This function
    returns another function for optimization reasons.

    gamma: double (controls how quickly agents can approach each other.  lower = slower)
    safety_radius: double (how far apart the agents will stay)
    magnitude_limit: how fast the robot can move linearly.

    -> function (the barrier certificate function)

    DECENTRALIZED VERSION

    CREATED BY: Robert Wilk
    LAST MODIFIED: 10/19/2022
    """

    # Check user input types

    # gamma = barrier_gain
    if URandSpan is None:
        URandSpan = [0]
    if XRandSpan is None:
        XRandSpan = [0]

    assert isinstance(gamma, (int,
                                     float)), "In the function create_single_integrator_barrier_certificate, the barrier gain (gamma) must be an integer or float. Recieved type %r." % type(
        gamma).__name__
    assert isinstance(safety_radius, (int,
                                      float)), "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be an integer or float. Recieved type %r." % type(
        safety_radius).__name__
    assert isinstance(magnitude_limit, (int,
                                        float)), "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be an integer or float. Recieved type %r." % type(
        magnitude_limit).__name__

    # Check user input ranges/sizes
    assert gamma > 0, "In the function create_single_integrator_barrier_certificate, the barrier gain (gamma) must be positive. Recieved %r." % gamma
    assert safety_radius >= 0.12, "In the function create_single_integrator_barrier_certificate, the safe distance between robots (safety_radius) must be greater than or equal to the diameter of the robot (0.12m) plus the distance to the look ahead point used in the diffeomorphism if that is being used. Recieved %r." % safety_radius
    assert magnitude_limit > 0, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be positive. Recieved %r." % magnitude_limit
    assert magnitude_limit <= 0.2, "In the function create_single_integrator_barrier_certificate, the maximum linear velocity of the robot (magnitude_limit) must be less than the max speed of the robot (0.2m/s). Recieved %r." % magnitude_limit


    def f(dxi, x, XRandSpan, URandSpan):
        # Check user input types
        assert isinstance(dxi,
                          np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the single-integrator robot velocity command (dxi) must be a numpy array. Recieved type %r." % type(
            dxi).__name__
        assert isinstance(x,
                          np.ndarray), "In the function created by the create_single_integrator_barrier_certificate function, the robot states (x) must be a numpy array. Recieved type %r." % type(
            x).__name__

        # Check user input ranges/sizes
        assert x.shape[
                   0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the single integrator robot states (x) must be 2 ([x;y]). Recieved dimension %r." % \
                            x.shape[0]
        assert dxi.shape[
                   0] == 2, "In the function created by the create_single_integrator_barrier_certificate function, the dimension of the robot single integrator velocity command (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % \
                            dxi.shape[0]
        assert x.shape[1] == dxi.shape[
            1], "In the function created by the create_single_integrator_barrier_certificate function, the number of robot states (x) must be equal to the number of robot single integrator velocity commands (dxi). Recieved a current robot pose input array (x) of size %r x %r and single integrator velocity array (dxi) of size %r x %r." % (
        x.shape[0], x.shape[1], dxi.shape[0], dxi.shape[1])

        # Initialize some variables for computational savings
        N = dxi.shape[1]
        num_constraints = int(comb(N, 2))
        A = np.zeros((num_constraints, 2 * N))
        b = np.zeros(num_constraints)
        H = sparse(matrix(2 * np.identity(2 * N)))

        # TODO: check that these are going down the correct axis
        if len(XRandSpan) == 1:
            XRandSpan = np.zeros(2, N)
        if len(URandSpan) == 1:
            URandSpan = np.zeros(2, N)


        count = 0
        for i in range(N - 1):
            for j in range(i + 1, N):
                # each should be 1 value
                max_dvij_x = np.linalg.norm([URandSpan[0, i] + URandSpan[0, j]], 2)
                max_dvij_y = np.linalg.norm([URandSpan[1, i] + URandSpan[1, j]], 2)

                # each should be 1 value
                max_dxij_x = np.linalg.norm([x[0, i] - x[0, j]], 2) + np.linalg.norm([XRandSpan[0, i] + XRandSpan[0, j]], 2)
                max_dxij_y = np.linalg.norm([x[1, i] - x[1, j]], 2) + np.linalg.norm([XRandSpan[1, i] + XRandSpan[1, j]], 2)

                b2_x, b1_x, sigma = trap_cdf_inv(XRandSpan[0, i], XRandSpan[0, j], x[0, i] - x[0, j], confidence_level)
                b2_y, b1_y, sigma = trap_cdf_inv(XRandSpan[1, i], XRandSpan[1, j], x[1, i] - x[1, j], confidence_level)

                if (b2_x < 0 and b1_x > 0) or (b2_x > 0 and b1_x < 0):
                    print('WARNING: distance between robots on x smaller than error bound!')
                    b_x = 0
                elif (b1_x < 0) and (b2_x < b1_x) or (b2_x < 0 and b2_x > b1_x):
                    b_x = b1_x
                elif (b2_x > 0 and b2_x < b1_x) or (b1_x > 0 and b2_x > b1_x):
                    b_x = b2_x
                else:
                    b_x = b1_x
                    print('WARNING: no uncertainty or sigma = 0.5 on x')  # b1 = b2 or no uncertainty

                if (b2_y < 0 and b1_y > 0) or (b2_y > 0 and b1_y < 0):
                    print('WARNING: distance between robots on y smaller than error bound!')
                    b_y = 0
                elif (b1_y < 0 and b2_y < b1_y) or (b2_y < 0 and b2_y > b1_y):
                    b_y = b1_y
                elif (b2_y > 0 and b2_y < b1_y) or (b1_y > 0 and b2_y > b1_y):
                    b_y = b2_y
                else:
                    b_y = b1_y
                    print('WARNING: no uncertainty or sigma = 0.5 on y')

                A[count, (2 * i)] = -2 * b_x  # matlab original: A(count, (2*i-1):(2*i)) = -2*([b_x;b_y]);
                A[count, (2 * i + 1)] = -2 * b_y

                A[count, (2 * j)] = 2 * b_x  # matlab original: A(count, (2*j-1):(2*j)) =  2*([b_x;b_y])';
                A[count, (2 * j + 1)] = 2 * b_y

                h1 = np.linalg.norm([b_x, 0.0]) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm([max_dvij_x, 0]) * np.linalg.norm([max_dxij_x, 0]) / gamma
                h2 = np.linalg.norm([0, b_y]) ** 2 - safety_radius ** 2 - 2 * np.linalg.norm([0, max_dvij_y]) * np.linalg.norm([0, max_dxij_y]) / gamma # h_y

                h = h1 + h2

                b[count] = gamma * h ** 3  # matlab original: b(count) = gamma*h^3
                count += 1

                '''
                OLD CODE

                # calculate error
                error = x[:, i] - x[:, j]
                h = (error[0] * error[0] + error[1] * error[1]) - np.power(safety_radius, 2)

                A[count, (2 * i, (2 * i + 1))] = -2 * error
                A[count, (2 * j, (2 * j + 1))] = 2 * error
                b[count] = barrier_gain * np.power(h, 3)

                count += 1
                '''

        # Threshold control inputs before QP
        norms = np.linalg.norm(dxi, 2, 0)
        idxs_to_normalize = (norms > magnitude_limit)
        dxi[:, idxs_to_normalize] *= magnitude_limit / norms[idxs_to_normalize]

        f_mat = -2 * np.reshape(dxi, 2 * N, order='F')
        result = qp(H, matrix(f_mat), matrix(A), matrix(b))['x']

        return np.reshape(result, (2, -1), order='F')

    return f


"""
this function constructs the trap distribution resultant from convolution of two different central uniform
distribution(i.e. from measurements of two robots positions)

    bot 1: uniformly distributed between[-a, a]
    bot 2: uniformly distributed between[-c, c]
    delta: x_bot1 - x_bot2 error between the two noisy measurements
    sigma: requred confidence level( > 50 %)
Output: 
    when sigma > .5
        b2: < b1 whose CDF corresponds to 1 - sigma
        b1: > b2 whose CDF corresponds to sigma
    when sigma < .5
        b2: > b1 whose CDF corresponds to 1 - sigma
        b1: < b2 whose CDF corresponds to sigma
        
FUNCTION FROM wenhaol ON GITHUB
ADAPTED TO PYTHON FROM MATLAB BY: Robert Wilk
LAST MODIFIED: 10/19/2022
"""
def trap_cdf_inv(a, c, delta, sigma):
    # returns list of b2, b1, sigma
    b2 = delta
    b1 = delta

    # a and c should be positive

    if a > c: # [-A, A] is the large one, and[-C, C] is the smaller one
        A = a
        C = c
    else:
        A = c
        C = a

    if A == 0 and C == 0:
        return b2, b1, sigma

    # O_vec = [-(A + C), -(A - C), (A - C), (A + C)] # vector of vertices on the trap distribution cdf

    h = 1 / (2 * A) # height of the trap distribution
    area_seq = [1/2 * 2 * C * h, 2 * (A - C) * h, 1/2 * 2 * C * h]
    area_vec = [area_seq[0], sum(area_seq[:2])]

    if abs(A - C) < 1e-5: # then is triangle
        # assuming sigma > 50
        b1 = (A + C) - 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1])) # 1 - area_vec[1] should be very close to 0.5
        b2 = -b1

        b1 = b1 + delta
        b2 = b2 + delta # apply shift here due to xi - xj

    else: # than is trap
        if sigma > area_vec[1]: # right triangle area
            b1 = (A + C) - 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1]))
            b2 = -(A + C) + 2 * C * np.sqrt((1 - sigma) / (1 - area_vec[1]))

            b1 = b1 + delta
            b2 = b2 + delta # apply shift here due to xi - xj

        elif sigma > area_vec[0] and sigma <= area_vec[1]: # in between the triangle part
            b1 = -(A - C) + (sigma - area_vec[0]) / h # assuming > 50%, then b1 should > 0
            b2 = -b1

            b1 = b1 + delta
            b2 = b2 + delta # apply shift here due to xi - xj

            # note that b1 could be > or < b2, depending on whether sigma > or < .5

        elif sigma <= area_vec[0]:
            b1 = -(A + C) + 2 * C * np.sqrt(sigma / area_vec[0]) # assuming > 50%, then b1 should > 0
            b2 = -b1

            b1 = b1 + delta
            b2 = b2 + delta # apply shift here due to xi - xj

        else:
            print('first triangle, which is not allowed as long as we assume sigma > 50%')

    return b2, b1, sigma
