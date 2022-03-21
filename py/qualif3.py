import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt # sensors filtering functions
import numpy as np
import time


if __name__ == "__main__":
    pseudo = "Clément Vellu"  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl() # create a robot controller
    irr_filt = filt.LowPassFilter()

    distancedroite = rb.get_sonar('right')
    distancegauche = rb.get_sonar('left')
    distancefront = rb.get_sonar('front')
    distanceback = rb.get_sonar('back')

    if distancegauche == 0 or (distancegauche > distancedroite != 0):
        print("Rotation gauche")
        ctrl.rotate_on_itself(rb, 90)
    else:
        print("Rotation droite")
        ctrl.rotate_on_itself(rb, -90)

    ctrl.go_straight_using_walls_stop_obstacle(rb, 150, 0.3, irr_filt)

    rb.set_speed(0, 0)

    distancedroite = rb.get_sonar('right')
    distancegauche = rb.get_sonar('left')
    distancefront = rb.get_sonar('front')
    distanceback = rb.get_sonar('back')

    if distancegauche == 0 or (distancegauche > distancedroite != 0):
        print("Rotation gauche")
        ctrl.rotate_on_itself(rb, 90)
    else:
        print("Rotation droite")
        ctrl.rotate_on_itself(rb, -90)

    ctrl.go_straight_using_walls_stop_obstacle(rb, 150, 0.3, irr_filt)

    rb.full_end()
