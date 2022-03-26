import numpy as np
import time


class LowPassFilter:

    def __init__(self): 
        self.a = 0.0  # here you define some useful variables
        self.mem = 0.0
        self.init = True
        self.old_value = []
    
    def irr(self, x, a):
        """
        Returns the mesurent xf which is the weighted average
        (factor a < 1) of the previous mesurement mem and the
        current mesurement x using the following formula :
        xf = x*a + (1-a)*mem

        If this is the first time this function is called, the
        mesurement is returned as it is
        :return: Filtered value
        """

        if self.init:
            self.mem = x
            self.init = False

        if x == 0:
            self.irr_reset()
            return 0

        xf = x*a + self.mem * (1-a)
        self.mem = xf
        return xf

    def irr_reset(self):
        """
        Resets the filter
        :return: Nothing
        """
        self.mem = 0.0
        self.init = True

    def moving_avg(self, x, n):
        if x == 0:
            self.old_value = []

        if len(self.old_value) < n:
            self.old_value.append(x)
        else:
            self.old_value.pop(0)
            self.old_value.append(x)

        avg = 0
        for elem in self.old_value:
            avg += elem
        return avg / len(self.old_value)

    def median_value(self, x, n):
        if x == 0:
            self.old_value = []

        if len(self.old_value) < n:
            self.old_value.append(x)
        else:
            self.old_value.pop(0)
            self.old_value.append(x)

        return np.median(self.old_value)

