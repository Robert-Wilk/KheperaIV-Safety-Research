import matplotlib.pyplot as plt
from sklearn.metrics.pairwise import euclidean_distances
import numpy as np
from utilities import util


class Telemetry:
    def __init__(self, agent_count, test_name, algorithm):
        self.agent_count = agent_count
        self.test_name = test_name
        self.algorithm = algorithm

        self.graphing_x = []
        self.graphing_y = []

        for i in range(self.agent_count):
            self.graphing_x.append([])
            self.graphing_y.append([])

        self.min_dist = []

        self.iterations = 0

    def update(self, x, positions):

        # add x and y to graph
        for i in range(self.agent_count):
            self.graphing_x[i].append(x[0, i])
            self.graphing_y[i].append(x[1, i])

        # calculate min inter-robot distance
        dist = np.triu(euclidean_distances(positions, positions))
        self.min_dist.append(min(dist[np.where(dist > 0)]))

        self.iterations += 1

    def __str__(self):
        return f'Telemetry Information for ${self.algorithm} on ${self.test_name} Test\n \
                Number of Agents: ${self.agent_count}\n \
                Current Iteration: ${self.iterations}\n'

    def create_graph(self, divider=1, min_dist=False, t_obj=None):
        # DIVIDER MUST BE FRACTION

        if not min_dist:

            if divider == 1:
                # set iterations to array index
                self.iterations -= 1

            for i in range(len(self.graphing_x)):
                plt.plot(self.graphing_x[i][:int(self.iterations * divider)],
                         self.graphing_y[i][:int(self.iterations * divider)],
                         label="Robot %s's position" % i,
                         linestyle='-', marker='.')
                color = plt.gca().lines[-1].get_color()
                plt.gca().add_patch(
                    plt.Circle((self.graphing_x[i][int(self.iterations * divider)],
                                self.graphing_y[i][int(self.iterations * divider)]),
                               0.05, facecolor='none', ec=color))

            plt.legend()
            plt.title('%s Test With %s | Test with %s Robots | Iteration: %s' %
                      (self.test_name, self.algorithm, self.agent_count, int(self.iterations * divider)))

            plt.show()
            util.wait_for_next()
            plt.clf()

            if divider == 1:
                # return iterations back to normal
                self.iterations += 1

        elif t_obj != None:
            plt.title('Minimum Inter-Robot Distance On %s | Test with %s Robots' % (self.test_name, self.agent_count))
            plt.plot([i for i in range(self.iterations)], self.min_dist,
                     label="Minimum Inter-Robot Distance PrSBC", linestyle='-',
                     marker='.')
            plt.plot([i for i in range(t_obj.iterations)], t_obj.min_dist,
                     label="Minimum Inter-Robot Distance %s" % t_obj.algorithm,
                     linestyle='-',
                     marker='.')
            plt.legend()
            plt.show()

