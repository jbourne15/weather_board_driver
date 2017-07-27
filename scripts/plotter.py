#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import time
from weather_board_driver.msg import wb_list

### Global variables.

plt.ion()
fsize=(14,12)
fig = plt.figure('humidity_plot', figsize=fsize)
#ax = fig.add_subplot(111, projection='3d')
ax = fig.add_subplot(111)#, projection='3d')
ax.grid(True)
plt.xlabel(r'$x$ '+ r'$(m)$', fontsize=20)
plt.ylabel(r'$y$ '+ r'$(m)$',fontsize=20)
#ax.set_zlim(30,100)


X = np.linspace(0,8,5)
Y = np.linspace(8,0,5)
X, Y = np.meshgrid(X, Y)
Z = None
surf = None
        
def getData_callback(data):
    global Z
    data = np.array(data.wb_list)
    Z = np.array([ [data[0].humidity, data[5].humidity, data[10].humidity, data[15].humidity, data[20].humidity],
                   [data[1].humidity, data[6].humidity, data[11].humidity, data[16].humidity, data[21].humidity],
                   [data[2].humidity, data[7].humidity, data[12].humidity, data[17].humidity, data[22].humidity],
                   [data[3].humidity, data[8].humidity, data[13].humidity, data[18].humidity, data[23].humidity],
                   [data[4].humidity, data[9].humidity, data[14].humidity, data[19].humidity, data[24].humidity] ])

    #data = np.reshape(data,(5,5)).T
    ###
    
def surface_updater():
    global X,Y,Z,fig,fsize,surf
    fig.clf()
    fig = plt.figure('humidity_plot', figsize=fsize)
    #ax = fig.add_subplot(111, projection='3d')
    ax = fig.add_subplot(111)#, projection='3d')
    #ax.set_zlim(30,100)
    ax.grid(True)
    plt.xlabel(r'$x$ '+ r'$(m)$', fontsize=20)
    plt.ylabel(r'$y$ '+ r'$(m)$',fontsize=20)

    #surf = ax.plot_surface(X, Y, Z)#, cmap='coolwarm')
    surf = ax.contourf(X, Y, Z,100, cmap='coolwarm')
    #ax.plot_wireframe(X, Y, Z,color='k')
    fig.canvas.draw()
    
#Init main
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("wb_list", wb_list, getData_callback)
    rate = rospy.Rate(100)    
    #import pdb; pdb.set_trace()
    
    while not rospy.is_shutdown():
        if Z is not None:
            surface_updater()
        else:
            print "No data recieved, start sensor_grid launch file"
            time.sleep(1)
            #rospy.loginfo_throttle(5, "No data recieved, start sensor_grid launch file")
        rate.sleep()
    #import pdb; pdb.set_trace()
