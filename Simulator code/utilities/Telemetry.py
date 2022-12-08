import matplotlib.pyplot as plt
from sklearn.metrics.pairwise import euclidean_distances
import numpy as np
from utilities import util


class Telemetry:
    def __init__(self, agent_count, test_name, algorithm, do_uBox=False, do_telemetry=True):
        if do_telemetry:
            self.agent_count = agent_count
            self.test_name = test_name
            self.algorithm = algorithm
            self.do_uBox = do_uBox

            self.graphing_x = []
            self.graphing_y = []
            if self.do_uBox:
                self.uncertain_x = []
                self.uncertain_y = []

            for i in range(self.agent_count):
                self.graphing_x.append([])
                self.graphing_y.append([])
                if self.do_uBox:
                    self.uncertain_x.append([])
                    self.uncertain_y.append([])

            self.min_dist = []
            self.minest_dist = 10
            self.minest_dist_index = 0

            self.iterations = 0
            self.do_telemetry = True

        else:
            self.do_telemetry = False

    def update(self, x, positions):
        if self.do_telemetry:
            # add x and y to graph
            for i in range(self.agent_count):
                self.graphing_x[i].append(positions.T[0, i])
                self.graphing_y[i].append(positions.T[1, i])
                if self.do_uBox:
                    self.uncertain_x[i].append(x[0, i])
                    self.uncertain_y[i].append(x[1, i])

            # calculate min inter-robot distance
            dist = np.triu(euclidean_distances(positions, positions))
            # print(euclidean_distances(positions, positions))
            self.min_dist.append(min(dist[np.where(dist > 0)]))
            # print(self.min_dist)

            if self.min_dist[self.iterations] < self.minest_dist:
                self.minest_dist = self.min_dist[self.iterations]
                self.minest_dist_index = self.iterations

            self.iterations += 1


    def __str__(self):
        if self.do_telemetry:
            return f'Telemetry Information for ${self.algorithm} on ${self.test_name} Test\n \
                    Number of Agents: ${self.agent_count}\n \
                    Current Iteration: ${self.iterations}\n'

    def create_graph(self, divider=1, min_dist=False, minest_dist=False, t_obj=None, error_bound_x=None, error_bound_y=None):
        if self.do_telemetry:
            # DIVIDER MUST BE FRACTION
            if not min_dist and not minest_dist:

                if divider == 1:
                    # set iterations to array index
                    self.iterations -= 1

                for i in range(len(self.graphing_x)):
                    # graphing_x/y is indexed agent, position
                    plt.plot(self.graphing_x[i][:int(self.iterations * divider)],
                             self.graphing_y[i][:int(self.iterations * divider)],
                             label="Robot %s's position" % i,
                             linestyle='-', marker='.', linewidth=1.0)
                    color = plt.gca().lines[-1].get_color()

                    if self.do_uBox:
                        error_bound_x2 = (error_bound_x + 2.0 * 0.2) / 2.0
                        error_bound_y2 = (error_bound_y + 2.0 * 0.2) / 2.0
                        plt.gca().add_patch(
                            plt.Circle((self.graphing_x[i][int(self.iterations * divider)],
                                        self.graphing_y[i][int(self.iterations * divider)]),
                                       0.14 / 2.0, facecolor='none', ec=color))
                        plt.gca().add_patch(
                            plt.Circle((self.uncertain_x[i][int(self.iterations * divider)],
                                        self.uncertain_y[i][int(self.iterations * divider)]),
                                       0.14 / 2.0, facecolor='none', linestyle='--', ec='k'))
                        plt.gca().add_patch(
                            plt.Rectangle((self.graphing_x[i][int(self.iterations * divider)] - (error_bound_x2[:, i] / 2.0),
                                           self.graphing_y[i][int(self.iterations * divider)] - (error_bound_y2[:, i] / 2.0)),
                                          error_bound_x2[:, i], error_bound_y2[:, i], facecolor='none', ec='magenta', linewidth=2))
                    else:
                        plt.gca().add_patch(
                            plt.Circle((self.graphing_x[i][int(self.iterations * divider)],
                                        self.graphing_y[i][int(self.iterations * divider)]),
                                       0.14 / 2.0, facecolor='none', ec=color))

                plt.legend()
                plt.title('%s Test With %s | Test with %s Robots | Iteration: %s' %
                          (self.test_name, self.algorithm, self.agent_count, int(self.iterations * divider)))

                plt.show()
                util.wait_for_next()
                plt.clf()

                if divider == 1:
                    # return iterations back to normal
                    self.iterations += 1

            elif minest_dist:

                for i in range(len(self.graphing_x)):
                    # graphing_x/y is indexed agent, position
                    plt.plot(self.graphing_x[i][:self.minest_dist_index],
                             self.graphing_y[i][:self.minest_dist_index],
                             label="Robot %s's position" % i,
                             linestyle='-', marker='.', linewidth=1.0)
                    color = plt.gca().lines[-1].get_color()

                    if self.do_uBox:
                        error_bound_x2 = (error_bound_x + 2.0 * 0.2) / 2.0
                        error_bound_y2 = (error_bound_y + 2.0 * 0.2) / 2.0
                        plt.gca().add_patch(
                            plt.Circle((self.graphing_x[i][self.minest_dist_index],
                                        self.graphing_y[i][self.minest_dist_index]),
                                       0.14 / 2.0, facecolor='none', ec=color))
                        plt.gca().add_patch(
                            plt.Circle((self.uncertain_x[i][self.minest_dist_index],
                                        self.uncertain_y[i][self.minest_dist_index]),
                                       0.14 / 2.0, facecolor='none', linestyle='--', ec='k'))
                        plt.gca().add_patch(
                            plt.Rectangle(
                                (self.graphing_x[i][self.minest_dist_index] - (error_bound_x2[:, i] / 2.0),
                                 self.graphing_y[i][self.minest_dist_index] - (error_bound_y2[:, i] / 2.0)),
                                error_bound_x2[:, i], error_bound_y2[:, i], facecolor='none', ec='magenta',
                                linewidth=2))
                    else:
                        plt.gca().add_patch(
                            plt.Circle((self.graphing_x[i][self.minest_dist_index],
                                        self.graphing_y[i][self.minest_dist_index]),
                                       0.05, facecolor='none', ec=color))

                plt.legend()
                plt.title('%s Test Smallest Distance Found %s | Iteration: %s' %
                          (self.test_name, self.minest_dist, self.minest_dist_index))

                plt.show()
                util.wait_for_next()
                plt.clf()
            elif type(t_obj) == list:
                smallest_iterations = 1000
                for i in range(len(t_obj)):
                    if t_obj[i].iterations < smallest_iterations:
                        smallest_iterations = t_obj[i].iterations
                if self.iterations < smallest_iterations:
                    smallest_iterations = self.iterations

                plt.title(
                    'Minimum Inter-Robot Distance On %s | Test with %s Robots' % (self.test_name, self.agent_count))
                plt.plot([i for i in range(smallest_iterations)], [0.4 for i in range(smallest_iterations)],
                         label="Minimum Inter-Robot Safety Distance", linestyle='--')

                plt.plot([i for i in range(smallest_iterations)], self.min_dist[:smallest_iterations],
                         label="Minimum Inter-Robot Distance For %s" % self.algorithm, linestyle='-',
                         marker='.')
                try:
                    for j in range(len(t_obj)):
                        plt.plot([i for i in range(smallest_iterations)], t_obj[j].min_dist[:smallest_iterations],
                                 label="Minimum Inter-Robot Distance For %s" % t_obj[j].algorithm,
                                 linestyle='-',
                                 marker='.')
                except:
                    print('Created Minimum Distance for Single test')
                finally:
                    plt.legend()
                    plt.show()

                    util.wait_for_next()
                    plt.clf()
            else:
                plt.title(
                    'Minimum Inter-Robot Distance On %s | Test with %s Robots' % (self.test_name, self.agent_count))
                plt.plot([i for i in range(self.iterations)], [0.4 for i in range(self.iterations)],
                         label="Minimum Inter-Robot Safety Distance", linestyle='--')

                plt.plot([i for i in range(self.iterations)], self.min_dist,
                         label="Minimum Inter-Robot Distance For %s" % self.algorithm, linestyle='-',
                         marker='.')
                try:
                    for j in range(len(t_obj)):
                        plt.plot([i for i in range(t_obj[j].iterations)], t_obj[j].min_dist,
                                 label="Minimum Inter-Robot Distance For %s" % t_obj[j].algorithm,
                                 linestyle='-',
                                 marker='.')
                except:
                    print('Created Minimum Distance for Single test')
                finally:
                    plt.legend()
                    plt.show()

                    util.wait_for_next()
                    plt.clf()