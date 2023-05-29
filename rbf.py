import numpy as np

class rbf():
    def __init__(self, input, input_dim, k):
        # input=(k*dim + 2*k + 1, )
        # m=(k, dim), std=(k, ), w=(k, ), b=(1, )
        self.m = input[:k*input_dim]
        self.m = np.reshape(self.m, (k, input_dim))
        self.std = input[k*input_dim:k*(input_dim+1)]
        self.weights = input[k*(input_dim+1):k*(input_dim+2)]
        self.bias = input[-1:]

    def forward(self, input):
        # input=(dim, ), m=(k, dim)
        self.input = input
        self.phi = np.exp(-1 * ((np.linalg.norm(self.input - self.m, axis=1))**2 / (2 * self.std**2)))
        # w=(k, ), phi=(k, ), b=(1, )
        return np.dot(self.weights, self.phi) + self.bias
