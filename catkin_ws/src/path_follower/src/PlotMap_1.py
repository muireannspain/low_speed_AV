#!/usr/bin/env python
#This is going to be the Python simulation node. Currently just a code to plot the trajectory of the modeled car
#as calculated by the MPC node in Julia, and the path follower code used as another node

# importing libraries
import matplotlib
#matplotlib.use('TkAgg',warn=False, force=True)
#import matplotlib.image as mpimg
import matplotlib.pyplot as plt
#matplotlib.use('GTK3Agg')
import numpy as np
#from PIL import Image
#import pandas as pd
import csv
import time
# import ipdb
#from IPython import display
import rospy
import math

#remove or add the message type
from std_msgs.msg import Float64MultiArray

received_data=[0,0,0,0,0,0]

def callback(data):
    global received_data
    # print rospy.get_name(), "I heard %f"%str(data.data)
    received_data=data.data
    # print(received_data)



# #open map image and resize to size of carpark
# image = Image.open('Map.png')
# # Read Images
# new_image = image.resize((300, 200))
# new_image.save('Map_resize.png')
# plt.imshow(new_image)

#load in MPC generated data for X, Y, velocity and heading at each point
#opened_file = open('Z.csv')
#from csv import reader
#read_file = reader(opened_file)
#Z = list(read_file)
#n=len(Z)
#z_list = [list(map(float,i)) for i in Z]
#z_list=np.array(z_list)

#opened_file = open('U.csv')
#read_file = reader(opened_file)
#u = list(read_file)
#n=len(u)
#u_list = [list(map(float,i)) for i in u]
#u_list=np.array(u_list)
#u=np.array(u_list)



opened_file = open('/home/uav/catkin_ws/src/path_follower/src/RealWaypoints.csv')
from csv import reader
read_file = reader(opened_file)
wayp = list(read_file)
n=len(wayp)
waypoints = [list(map(float,i)) for i in wayp]
waypoints=np.array(waypoints)

def plotting_func(data):
    #returns the next x,y coordinates of the car
    # Parameters of car

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
    global received_data
    #to animate the figure
    i=1
    fig = plt.figure()
    ax = plt.gca()
    plt.ion()


    print("Plt show")

    z0=np.array(received_data[:4])
    u=np.array(received_data[4:])

    obj, wheels=plotting_func(received_data)


    h1 = ax.plot(obj[0], obj[4],'b-')[0]
    h2 = ax.plot(obj[1], obj[5], 'b-')[0]
    h3 = ax.plot(obj[2], obj[6], 'b-')[0]
    h4 = ax.plot(obj[3], obj[7], 'b-')[0]
    h5=ax.plot([wheels[0],wheels[1]],[wheels[4],wheels[5]],'r-')[0]
    h7=ax.plot([wheels[2],wheels[3]],[wheels[6],wheels[7]],'r-')[0]
    h9 = ax.plot(waypoints[:,0], waypoints[:,1], 'mo')

    plt.show()
    tic = time.time()

    while not rospy.is_shutdown():

        print(received_data)
        # update the xy data
        obj,wheels = plotting_func(received_data)
        h1.set_data(obj[0], obj[4])
        h2.set_data(obj[1], obj[5])
        h3.set_data(obj[2], obj[6])
        h4.set_data(obj[3], obj[7])
        h5.set_data([wheels[0],wheels[1]],[wheels[4],wheels[5]])
        h7.set_data([wheels[2],wheels[3]],[wheels[6],wheels[7]])

        start = time.time()
        i=i+1

        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()



def listener():
    rospy.init_node('listener_1')
    sub=rospy.Subscriber("pts", Float64MultiArray, callback)
    # print("listener")

if __name__ == '__main__':
    try:
        listener()
        # rospy.spin()
        run()
    except rospy.ROSInterruptException:
        pass
