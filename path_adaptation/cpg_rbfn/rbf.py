import numpy as np

class RBF:
    def __init__(self, nc = 30, variance_gaussian = 0.01, nM = 500, alpha = 0.25): 
        self.nc = nc
        self.variance_gaussian = variance_gaussian
        self.nM = nM
        self.alpha = alpha

        self.target_length = None
        self.ci = []
        self.M  = []
        self.M_stack = []

        self.learning_iteration = 0
        self.learning_rate = 0.25

    def construct_kernels_with_cpg(self, O0_cpg_one_cycle, O1_cpg_one_cycle, target_length):
        
        if target_length is None:
            print('please input target length')
            return 

        self.target_length = target_length

        self.K = np.zeros((self.nc, self.target_length))
        self.W = np.zeros((self.nc))
        self.M = np.zeros((self.target_length))

        self.ci = np.linspace(0, O0_cpg_one_cycle.shape[0]-1, num = self.nc, dtype = int) # self.ci is the indices of raw data center at each sampling point
        cx = O0_cpg_one_cycle[self.ci]
        cy = O1_cpg_one_cycle[self.ci]

        b = np.zeros((self.target_length, 1))
        for i in range(self.nc):
            b = np.exp(-(np.power((O0_cpg_one_cycle - cx[i]), 2) + np.power((O1_cpg_one_cycle - cy[i]), 2)) / self.variance_gaussian) # b is a normalized gaussian distribution
            self.K[i, :] = b.transpose()
        return self.K
    
    def calculate_RBF_weight(self, target_traj, learning_iteration = 500, learning_rate= 0.25):
        self.learning_iteration = learning_iteration
        self.learning_rate = learning_rate
        learning_iteration = learning_iteration
        for i in range(learning_iteration):
            self.M = np.matmul(self.W, self.K)           # calculate forward value <--- Inspect these iterations
            self.W = self.W + learning_rate * (target_traj[self.ci] - self.M[self.ci])
            self.M_stack.append(self.M)