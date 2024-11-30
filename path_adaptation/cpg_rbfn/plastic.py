import numpy as np

class Plastic:
    def __init__(self, o0, o1, o2, initial_phi, _alpha, lrate):
        self.out0_t = o0
        self.out1_t = o1
        self.out2_t = o2
        self.phi =  initial_phi
        self.alpha = _alpha
        self.w20_t = 0  # w -> weight
        self.w02_t = 1 # Default = 1
        self.w2p_t = 0.03
        self.learning = lrate # default = 1.3
        self.counter = 0
        
        # ----------- Default -----------
        factor = 1
        self.A02 = 1*factor
        self.A20 = 1*factor
        self.A2p = 1
        self.B02 = 0.01*factor
        self.B20 = 0.01*factor
        self.B2p = 0.01*factor
        
        self.z = 0
        self.beta = 0.5
        
        self.w00 = self.alpha*np.cos(self.phi)
        self.w01 = self.alpha*np.sin(self.phi)
        self.w10 = self.alpha*(-np.sin(self.phi))
        self.w11 = self.alpha*np.cos(self.phi)
        

        self.discretize_count = 0
        self.discretize_factor = 10

        self.out0       = []
        self.out1       = []
        self.outFreq    = []

    def updateWeights(self):
        # ----------- SGD ----------- stocastic gradient descent (new = old + (lrate*error))
        e = 1
        self.phi = self.phi + self.learning*self.w02_t*self.out2_t*self.w01*self.out1_t*e

        # ----------- Momentum -----------
        # e = -1
        # self.z = self.beta*self.z + self.w02_t*self.out2_t*self.w01*self.out1_t
        # self.phi = self.phi - self.learning*self.z*e
        # ----------- update phi -----------
        self.w00 = self.alpha*np.cos(self.phi)
        self.w01 = self.alpha*np.sin(self.phi) 
        self.w10 = self.alpha*(-np.sin(self.phi)) 
        self.w11 = self.alpha*np.cos(self.phi)  

    def update_cpg(self, set_fcpg, discretize_factor):

        self.discretize_factor = discretize_factor
        phi = set_fcpg*2*3.14
        if self.discretize_count % self.discretize_factor == 0.0:
            self.setPhi(phi)
            self.update(perturbation=0.0)
        self.discretize_count += 1

    def gen_cpg_finite_size(self, cpg_length = 100000):
        sizeOut         = cpg_length                # increase cpg list size and decrease 
        self.out0       = np.empty((1,sizeOut))
        self.out1       = np.empty((1,sizeOut))
        self.outFreq    = np.empty((1,sizeOut))

        for idx in range(sizeOut):
            self.update_cpg(0.001, 10)                  # adjust following to sample frequency
            self.out0[0][idx]    = self.getOut0() 
            self.out1[0][idx]    = self.getOut1()
            self.outFreq[0][idx] = self.getFrequency() 
        return {'out0':self.out0[0],
                'out1':self.out1[0]}

    def gen_cpg_one_cycle(self):
        self.gen_cpg_finite_size()
        cpg_cycle_index = self.zero_crossing_one_period(self.out0[0])

        out0_cpg_one_cycle = self.out0[0][cpg_cycle_index[0]:cpg_cycle_index[1]]
        out1_cpg_one_cycle = self.out1[0][cpg_cycle_index[0]:cpg_cycle_index[1]]

        return {'out0_cpg_one_cycle': out0_cpg_one_cycle,
                'out1_cpg_one_cycle': out1_cpg_one_cycle}

    def zero_crossing_one_period(self, signal):
        """
        Extract specific cycles (from start_cycle to end_cycle, inclusive) of a signal.

        Parameters:
            signal (numpy array): The input signal array.
            start_cycle (int): The first cycle to extract.
            end_cycle (int): The last cycle to extract.

        Returns:
            numpy array: The portion of the signal corresponding to the specified cycles,
                         or None if the cycles don't exist.
        """
        start_cycle = 3 
        end_cycle = 4
        
        # Identify the sign of the signal
        sign_signal = np.sign(signal)

        # Find zero-crossing indices
        crossings = np.where(np.diff(sign_signal) != 0)[0]

        # Check if enough crossings exist
        if len(crossings) < end_cycle:
            return None  # Not enough zero crossings to extract the requested cycles

        # Extract the indices for the requested cycles
        start_index = crossings[start_cycle - 1]
        end_index = crossings[end_cycle]

        # Return the signal corresponding to the specified cycles
        return [start_index, end_index + 1]

    
    def getOut0(self):
        return self.out0_t
    
    def getOut1(self):
        return self.out1_t
    
    def getOut2(self):
        return self.out2_t
    
    def setPhi(self, newPhi):
        self.phi = newPhi
        
    def getFrequency(self):
        if (self.alpha != 1.01):
            return -1
        else:
            return self.phi/(2*np.pi)  # return fcpg
    
    def getW2p(self):
        return self.w2p_t
    
    def getW20(self):
        return self.w20_t
    
    def getW02(self):
        return self.w02_t
    
    def getW00(self):
        return self.w00
    
    def getW01(self):
        return self.w01
    
    def getW10(self):
        return self.w10
    
    def getW11(self):
        return self.w11
    
    def print_all_param(self):
        print("out0_t: {}".format(self.out0_t))
        print("out1_t: {}".format(self.out1_t))
        print("out2_t: {}".format(self.out2_t))
        print("freq: {}".format(self.phi/(2*3.14)))
        print("alpha: {}".format(self.alpha))
        print("w20_t: {}".format(self.w20_t))
        print("w02_t: {}".format(self.w02_t))
        print("w2p_t: {}".format(self.w2p_t))
        print("learning: {}".format(self.learning))
        print("A02: {}".format(self.A02))
        print("A20: {}".format(self.A20))
        print("A2p: {}".format(self.A2p))
        print("B02: {}".format(self.B02))
        print("B20: {}".format(self.B20))
        print("B2p: {}".format(self.B2p))


    def update(self, perturbation):
        w20_init = 0
        w02_init = 1
        w2p_init = 0.03 

        self.out0_t1 = np.tanh(self.w00*self.out0_t + self.w01*self.out1_t + self.w02_t*self.out2_t)
        self.out1_t1 = np.tanh(self.w10*self.out0_t + self.w11*self.out1_t)
        self.out2_t1 = np.tanh(self.w20_t*self.out0_t + self.w2p_t*(perturbation))  

        self.updateWeights()    
        
        self.w20_t1 = self.w20_t - self.A20*self.out2_t*self.out0_t - self.B20*(self.w20_t - w20_init)
        self.w02_t1 = self.w02_t - self.A02*self.out0_t*self.out2_t - self.B02*(self.w02_t - w02_init)
        self.w2p_t1 = self.w2p_t + self.A2p*self.out2_t*perturbation - self.B2p*(self.w2p_t - w2p_init) 
        self.out0_t = self.out0_t1
        self.out1_t = self.out1_t1
        self.out2_t = self.out2_t1  
        self.w20_t = self.w20_t1
        self.w02_t = self.w02_t1
        self.w2p_t = self.w2p_t1