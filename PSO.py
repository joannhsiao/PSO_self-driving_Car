import random
import numpy as np

class PSO():
    def __init__(self, particles):
        self.x = particles
        self.group_size = particles.shape[0]
        self.len_particle = particles.shape[1]
        self.p = particles
        self.v = np.zeros((self.group_size, self.len_particle))
        self.v_Max = 10

    def velocity_update(self, phi_1, phi_2):
        for i in range(self.group_size):
            for j in range(self.len_particle):
                r1 = random.random()
                r2 = random.random()
                cognitive = phi_1 * r1 * (self.p[i][j] - self.x[i][j])
                social = phi_2 * r2 * (self.best_position[j] - self.x[i][j])
                self.v[i][j] = self.v[i][j] + cognitive + social

            # cognitive = self.p[i] - self.x[i]
            # social = self.best_position - self.x[i]
            # self.v[i] += phi_1 * cognitive + phi_2 * social

                if self.v[i][j] > self.v_Max:
                    self.v[i][j] = self.v_Max
                elif self.v[i][j] < -self.v_Max:
                    self.v[i][j] = -self.v_Max

    def position_update(self):
        for i in range(self.group_size):
            self.x[i] += self.v[i]

    def evaluate_fitness(self, scores, iteration):
        index = np.where(scores[:, iteration]==max(scores[:, iteration]))[0][0]
        self.best_position = self.x[index]

        for i in range(self.group_size):
            index = np.where(scores[i]==max(scores[i]))[0][0]
            if index == iteration:
                self.p[i] = self.x[i]
