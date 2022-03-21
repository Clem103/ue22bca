import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt # sensors filtering functions
import numpy as np
import time


if __name__ == "__main__":
    pseudo = ""  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl() # create a robot controller
    irr_filt = filt.LowPassFilter()

    ctrl.go_straight_using_walls_stop_obstacle(rb, 30, 0.3, irr_filt)

    # put your mission code here

    rb.full_end()
