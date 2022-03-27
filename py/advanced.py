import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt # sensors filtering functions
import numpy as np
import time


if __name__ == "__main__":
    pseudo = ""  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl() # create a robot controller
    filt_left = filt.LowPassFilter()
    filt_right = filt.LowPassFilter()
    filt_front = filt.LowPassFilter()

    ctrl.follow_path_advanced(rb, 70, 0.2, filt_left, filt_right, filt_front)

    rb.full_end()
