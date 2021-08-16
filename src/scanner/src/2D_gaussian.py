#!/usr/bin/env python2

import rospy
import std_msgs.msg
import detection_msgs.msg
from std_msgs.msg import Float64, Int8

import numpy as np
from math import sqrt
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mod_var = 1.0
min = 100.0
prev_min = 100.0

class SetMaxSpeed:
    def __init__(self, max_speed_pub, det_pub, distance_pub, modify_var_pub):
        self.max_speed_pub = max_speed_pub
        self.det_pub = det_pub
        self.distance_pub = distance_pub
        self.modify_var_pub = modify_var_pub

    def symmetric_gaussian(self, pos, mu, Sigma):
        """Return the multivariate Gaussian distribution on array pos."""

        n = mu.shape[0]
        Sigma_det = np.linalg.det(Sigma)
        Sigma_inv = np.linalg.inv(Sigma)
        N = np.sqrt((2*np.pi)**n * Sigma_det)
        # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
        # way across all the input variables.
        fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

        return np.exp(-fac / 2) / N

    def social_speed(self, dx, dy, MAX_SPEED):
        x = np.arange(-3, 3, 0.1)
        y = np.arange(-3, 3, 0.1)
        X, Y = np.meshgrid(x, y)

        # Mean vector and covariance matrix
        mu = np.array([0.0, 0.0])
        Sigma = np.array([[ 0.5, 0.0 ], [ 0.0, 0.5 ]])

        # Pack X and Y into a single 3-dimensional array
        pos = np.empty(X.shape + (2,))
        pos[:, :, 0] = X
        pos[:, :, 1] = Y

        # The distribution on the variables X, Y packed into pos.
        Z = self.symmetric_gaussian(pos, mu, Sigma)

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.plot_surface(X, Y, Z, rstride=3, cstride=3, linewidth=1, antialiased=True, cmap=cm.viridis)
        # plt.contour(X, Y, Z)
        # plt.show()

        global mod_var
        if 1.0 - Z[29+dx, 29+dy] / Z[29, 29] < 0.999 :
            if mod_var - (1.0 - Z[29+dx, 29+dy] / Z[29, 29]) > 0.05 :
                mod_var = 1.0 - Z[29+dx, 29+dy] / Z[29, 29] + 0.02
            else :
                mod_var = 1.0 - Z[29+dx, 29+dy] / Z[29, 29]
        else :
            if mod_var > 0.4 :
                mod_var = mod_var - 0.01
            else :
                mod_var = mod_var - 0.02

        final_max_speed = mod_var * MAX_SPEED
        # rospy.logwarn("modify velocity to %lf", final_max_speed)
        self.max_speed_pub.publish(final_max_speed)
        self.modify_var_pub.publish(mod_var)

    def get_distance_callback(self, msg):
        global min
        global prev_min
        target = -1
        
        if len(msg.dets_list) != 0 :
            for i in range(len(msg.dets_list)) :
                min = msg.dets_list[i].x * msg.dets_list[i].x + msg.dets_list[i].y * msg.dets_list[i].y
                target = i
        
        if target != -1 :
            if min > prev_min :
                min = min - 0.05
            if min < 2.25 :
                self.social_speed(30, 30, 0.0)
            else :
                self.social_speed(int(msg.dets_list[target].x/0.2), int(msg.dets_list[target].y/0.2), 0.4)
        else :
            if min == prev_min and min > 0.3:
                min = min - 0.2
            if min < 2.25 :
                self.social_speed(30, 30, 0.0)
            else :
                self.social_speed(30, 30, 0.4)

        self.det_pub.publish(len(msg.dets_list))   
        self.distance_pub.publish(sqrt(min))
        prev_min = min

def main():
    rospy.init_node("Setting_Max_Speed")
    max_speed_pub = rospy.Publisher("/navigation_controller/max_speed", Float64, queue_size=10)
    det_pub = rospy.Publisher("/people/num", Int8, queue_size=10)
    distance_pub = rospy.Publisher("/people/distance", Float64, queue_size=10)
    modify_var_pub = rospy.Publisher("/modify_variable", Float64, queue_size=10)
    modifier = SetMaxSpeed(max_speed_pub, det_pub, distance_pub, modify_var_pub)
    sub = rospy.Subscriber("/scan_person_clustering_front_node/det3d_result", detection_msgs.msg.Det3DArray, modifier.get_distance_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
    