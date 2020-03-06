#!/usr/bin/env python

# importing libraries
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
import rospy
import math

#remove or add the message type
#from std_msgs.msg import Float64MultiArray
from cloud_msgs.msg import mpc_msg
from cloud_msgs.msg import convert_msg
#from nav_msgs.msg import Odometry

received_data_MPC=[0,0,0,0,0,0]
#x,y,v,head from conversion node
received_data_localization=[0,0,0,0]
OL_plotX=[0,0,0,0,0,0,0,0,0,0,0]
OL_plotY=[0,0,0,0,0,0,0,0,0,0,0]

def callbackMPC(data):
    global received_data_MPC
    global OL_plotX
    global OL_plotY
    OL_plotX=data.pre_x
    OL_plotY=data.pre_y
    received_data_MPC=data.z_u

def callbackLocalization(data):
    # global received_data_localization
    # received_data_localization=[data.x,data.y]
    received_data_localization[0] = data.x
    received_data_localization[1] = data.y
    received_data_localization[2] = data.v
    received_data_localization[3] = data.hd


    #print("loc sub", received_data_localization)


opened_file = open('/home/uav/catkin_ws/src/path_follower/src/xy_localization.csv')
from csv import reader
read_file = reader(opened_file)
wayp = list(read_file)
n=len(wayp)
waypoints = [list(map(float,i)) for i in wayp]
waypoints=np.array(waypoints)

def plotting_func(data):

    lf = 1.738
    lr = 1.738
    width = 2
    # Assume the length of the wheel is 0.5
    lw = 0.5
    #figure
    # front
    x_f = data[0]+lf*np.cos(data[3])
    y_f = data[1]+lf*np.sin(data[3])
    # front left
    x_f_l = x_f-(width/2)*np.sin(data[3])
    y_f_l = y_f+(width/2)*np.cos(data[3])
    # front right
    x_f_r = x_f+(width/2)*np.sin(data[3])
    y_f_r = y_f-(width/2)*np.cos(data[3])
    # rear
    x_r = data[0]-lr*np.cos(data[3])
    y_r = data[1]-lr*np.sin(data[3])
    # rear left
    x_r_l = x_r-(width/2)*np.sin(data[3])
    y_r_l = y_r+(width/2)*np.cos(data[3])
    # rear right
    x_r_r = x_r+(width/2)*np.sin(data[3])
    y_r_r = y_r-(width/2)*np.cos(data[3])

    x_d1=[x_f_l,x_f_r]
    x_d2=[x_f_l,x_r_l]
    x_d3=[x_r_l,x_r_r]
    x_d4=[x_f_r,x_r_r]
    y_d1=[y_f_l,y_f_r]
    y_d2=[y_f_l,y_r_l]
    y_d3=[y_r_l,y_r_r]
    y_d4=[y_f_r,y_r_r]

    # calculate the wheel
    # left wheel
    w_f_l_x = x_f_l+(lw/2)*np.cos(data[3]-data[5])
    w_r_l_x = x_f_l-(lw/2)*np.cos(data[3]-data[5])
    w_f_l_y = y_f_l+(lw/2)*np.sin(data[3]-data[5])
    w_r_l_y = y_f_l-(lw/2)*np.sin(data[3]-data[5])
    # right wheel
    w_f_r_x = x_f_r+(lw/2)*np.cos(data[3]-data[5])
    w_r_r_x = x_f_r-(lw/2)*np.cos(data[3]-data[5])
    w_f_r_y = y_f_r+(lw/2)*np.sin(data[3]-data[5])
    w_r_r_y = y_f_r-(lw/2)*np.sin(data[3]-data[5])

    # Draw the wheel
    obj=np.array([x_d1,x_d2,x_d3,x_d4,y_d1,y_d2,y_d3,y_d4])
    wheels=np.array([w_f_l_x,w_r_l_x,w_f_r_x,w_r_r_x,w_f_l_y,w_r_l_y,w_f_r_y,w_r_r_y])
    return obj, wheels


def run():
    global received_data_MPC
    global received_data_localization
    global OL_plotX
    global OL_plotY
    #to animate the figure
    i=1
    fig = plt.figure()
    ax = plt.gca()
    plt.ion()


    print("Plt show")

    #using [x,y] from localization node and [velocity, heading][acceeration, steering angle] from MPC
    #z0=np.array(received_data[:4])
    #u=np.array(received_data[4:])
    obj, wheels=plotting_func([0,0,0,0,0,0])


    h1 = ax.plot(obj[0], obj[4],'b-')[0]
    h2 = ax.plot(obj[1], obj[5], 'b-')[0]
    h3 = ax.plot(obj[2], obj[6], 'b-')[0]
    h4 = ax.plot(obj[3], obj[7], 'b-')[0]
    h5=ax.plot([wheels[0],wheels[1]],[wheels[4],wheels[5]],'r-',linewidth=3)[0]
    h7=ax.plot([wheels[2],wheels[3]],[wheels[6],wheels[7]],'r-',linewidth=3)[0]
    h9 = ax.plot(waypoints[:,0], waypoints[:,1], 'm.')
    h10=ax.plot(OL_plotX,OL_plotY,'g-',linewidth=3)[0]

    plt.show()
    tic = time.time()

    while not rospy.is_shutdown():
        received_data=[received_data_localization[0],received_data_localization[1],received_data_localization[2],received_data_localization[3],received_data_MPC[4],received_data_MPC[5]]

        print(received_data)
        # update the xy data
        obj,wheels = plotting_func(received_data)
        h1.set_data(obj[0], obj[4])
        h2.set_data(obj[1], obj[5])
        h3.set_data(obj[2], obj[6])
        h4.set_data(obj[3], obj[7])
        h5.set_data([wheels[0],wheels[1]],[wheels[4],wheels[5]])
        h7.set_data([wheels[2],wheels[3]],[wheels[6],wheels[7]])
        h10.set_data(OL_plotX,OL_plotY)

        start = time.time()
        i=i+1

        ax.relim()
        ax.autoscale_view()
        ax.axis("equal")
        fig.canvas.draw()



def listener_MPC():

    sub=rospy.Subscriber("mpc", mpc_msg, callbackMPC)
    # print("listener")

def listener_localization():
    sub=rospy.Subscriber("conversion", convert_msg, callbackLocalization)
    # print("listener")


if __name__ == '__main__':
    try:
        rospy.init_node('listener_1')
        listener_MPC()
        listener_localization()
        # rospy.spin()
        run()
    except rospy.ROSInterruptException:
        pass
