import numpy as np
import time

class RobotControl:
    def __init__(self):
        # set some useful constants
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 512
        self.wheelDiameter = 0.06
        self.data_dump = []


    def test_move (self,rb,speed_left,speed_right,duration):
        """
        Example of test function to check if the robot moves

        input parameters :
        rb : robot object
        speed_left : speed command of left wheel
        speed_right : speed command of right wheel
        duration : duration of the move

        output paremeters :
        None
        """
        # forward motion
        rb.set_speed(speed_left,speed_right)
        loopIterTime = 0.050
        tStart = time.time()
        while time.time()-tStart < duration:
            time.sleep(loopIterTime) # wait
        # stop the robot 
        rb.stop()

    def go_straight_stop_on_front_obstacle(self,rb,spd,max_dist):
        loop_iteration_time = 0.2
        rb.set_speed(spd,spd)
        state = True
        while state:
            t0_loop = time.time()
            distance = rb.get_sonar('front')
            if distance < max_dist and distance != 0:
                rb.set_speed(0, 0)
                state = False
            t_loop = time.time() - t0_loop
            if t_loop < loop_iteration_time:
                time.sleep(loop_iteration_time-t0_loop)

    def rotate_on_itself(self,rb,angle): # angle en degré (positif dans la rotation vers la gauche)
        loop_iteration_time = 0.2
        rayon_cercle = 0.06 # en m
        rayon_roue = 0.03 # en m
        rad = abs(2*np.pi*angle/360)
        d_virage = rad*rayon_cercle # distance que la roue doit parcourir pour faire tourner le robot de l'angle
        distance_roue_tick = 2*np.pi*rayon_roue/512
        nb_tick = np.floor(d_virage/distance_roue_tick)
        state = True
        left_t0, right_t0 = rb.get_odometers()
        while state:
            t0_loop = time.time()
            left, right = rb.get_odometers()
            if angle <= 0:
                if left - left_t0 < nb_tick * 0.80:
                    rb.set_speed(20, -20)
                else:
                    rb.set_speed(5, -5)
                if left - left_t0 >= nb_tick:
                    rb.set_speed(0, 0)
                    state = False
            else:
                if right - right_t0 < nb_tick * 0.80:
                    rb.set_speed(-20, 20)
                else:
                    rb.set_speed(-5, 5)
                if right - right_t0 > nb_tick:
                    rb.set_speed(0, 0)
                    state = False
            t_loop = time.time() - t0_loop
            if t_loop < loop_iteration_time:
                time.sleep(loop_iteration_time-t_loop)

    def go_straight_using_walls_stop_obstacle(self, rb, spd, max_dist, low_filter, track_width = 0.55):
        loop_iteration_time = 0.2
        rb.set_speed(spd, spd)
        state = True
        v_lat = 0
        old_d_left, old_d_right, old_v_lat = 0,0,0
        Kp = 10
        delta_heading = 0.01

        while state:
            t0_loop = time.time()
            rb.set_speed(spd, spd)
            raw_filt_distance_right = low_filter.irr(rb.get_sonar('right'), 0.70)
            raw_filt_distance_left = low_filter.irr(rb.get_sonar('left'), 0.70)
            raw_distance_front = rb.get_sonar('front')
            raw_filt_distance_back = low_filter.irr(rb.get_sonar('back'), 0.70)

            distance_right = raw_filt_distance_right + 0.04 if raw_filt_distance_right != 0 else raw_filt_distance_right    # distance entre le mur et le centre du robot à droite
            distance_left = raw_filt_distance_left + 0.04 if raw_filt_distance_left != 0 else raw_filt_distance_left      # distance entre le mur et le centre du robot à gauche

            distance_from_center_left = abs(1/2 * track_width - distance_left)  # Distance au centre basée sur le capteur gauche
            distance_from_center_right = abs(1/2 * track_width - distance_right)    # Distance au centre basée sur la capteur droite

            v_lat = 1/2 * ((old_d_left - distance_left) + (distance_right - old_d_right)) # Comptée positive lorsque l'on s'approche à gauche

            # if not turning_left and ((distance_left - old_d_left < - delta_heading and distance_left != 0) or (distance_right - old_d_right > delta_heading and distance_right != 0)):
            #     turning_left = True
            #     turning_right = False
            #     print("Heading left")
            # elif not turning_right and ((distance_right - old_d_right < - delta_heading and distance_right != 0) or (distance_left - old_d_left > delta_heading and distance_left != 0)):
            #     turning_left = False
            #     turning_right = True
            #     print("Heading right")

            # if distance_left != 0 and distance_right != 0:
            #     print('Correction traj en cours\nDistance droite : {}\ndistance gauche : {}'.format(distance_right, distance_left))
            #     if distance_left > distance_right:
            #         rb.set_speed(spd, spd + (distance_from_center_left * Kp))   # Tourner à gauche prop à l'écart entre le centre de la piste et le robot
            #     elif not turning_right:
            #         rb.set_speed(spd + (distance_from_center_right * Kp), spd)  # Tourner à droite prop à l'écart entre le centre de la piste et le robot
            #
            # elif distance_right == 0 or distance_right > 0.7:
            #     print("Pas de mur à droite !")
            #     if not turning_right and distance_left < 0.1:
            #         rb.set_speed(spd + distance_from_center_left * Kp, spd)  # Tourner à droite
            #     elif not turning_left and distance_left > 0.25:
            #         rb.set_speed(spd, spd + distance_from_center_left * Kp)  # Tourner à gauche
            #
            # elif distance_left == 0 or distance_left > 0.7:
            #     print("Pas de mur à gauche")
            #     if not turning_right and distance_right > 0.25:
            #         rb.set_speed(spd + (distance_from_center_right * Kp), spd)  # Tourner à droite
            #     elif not turning_left and distance_right < 0.1:
            #         rb.set_speed(spd, spd + (distance_from_center_right * Kp))  # Tourner à gauche

            if raw_distance_front != 0 and raw_distance_front < max_dist:
                print("Obstacle detected, front")
                rb.set_speed(0, 0)
                state = False
            old_d_right = distance_right
            old_d_left = distance_left
            old_v_lat = v_lat

            t_loop = time.time() - t0_loop
            if t_loop < loop_iteration_time:
                time.sleep(loop_iteration_time - t_loop)

    def follow_white_line_until_wall(self, rb, spd, max_dist):
        loop_iteration_time = 0.2
        print("Following white line")
        state = True
        rb.set_speed(spd,spd)
        seq_line = 'n'
        seq_line_old = 'n'
        old_value = (False, False, False)
        while state:
            t0_loop = time.time()

            left0, middle0, right0 = old_value
            left, middle, right = rb.get_centerline_sensors()
            left = left > 0.8
            middle = middle > 0.8
            right = right > 0.8
            if left != left0 or right != right0 or middle != middle0:
                if left0:
                    if middle:
                        print('Drift à gauche')
                        seq_line = 'g'
                    elif not left:
                        print('Drift à droite')
                        seq_line = 'd'
                elif middle0:
                    if right:
                        print('Drift à gauche')
                        seq_line = 'g'
                    if left:
                        print('Drift à droite')
                        seq_line = 'd'
                elif right0:
                    if middle:
                        print('Drift à droite')
                        seq_line = 'd'
                    elif not right:
                        print('Drift à gauche')
                        seq_line = 'g'

            old_value = (left, middle, right)

            if seq_line == 'g' and seq_line!=seq_line_old:
                rb.set_speed(spd*1.1, spd*0.9)
            elif seq_line == 'd' and seq_line!=seq_line_old:
                rb.set_speed(spd*0.9, spd*1.1)
            else:
                seq_line_old = seq_line

            if rb.get_sonar('front') != 0 and rb.get_sonar('front') < max_dist:
                state = False

            t_loop = time.time() - t0_loop
            print(t_loop)
            if t_loop < loop_iteration_time:
                time.sleep(loop_iteration_time - t_loop)






