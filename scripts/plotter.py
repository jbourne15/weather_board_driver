#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np
import time
from weather_board_driver.msg import wb_list
plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'

### Global variables.
fsize=(15,12)
fig = plt.figure('humidity_plot', figsize=fsize)
ax = fig.add_subplot(111)
ax.grid(True)
plt.axis([-1,9,-1,9])
plt.xlabel(r'$x$ '+ r'$(m)$', fontsize=20)
plt.ylabel(r'$y$ '+ r'$(m)$',fontsize=20)

isNewData = False

X = np.linspace(0,8,5)
Y = np.linspace(8,0,5)
X, Y = np.meshgrid(X, Y)
Z = None
###
        
def getData_callback(data):
    global Z, isNewData, X, Y
    data = np.array(data.wb_list)
    Z = np.array([ [data[0].humidity, data[5].humidity, data[10].humidity, data[15].humidity, data[20].humidity],
                   [data[1].humidity, data[6].humidity, data[11].humidity, data[16].humidity, data[21].humidity],
                   [data[2].humidity, data[7].humidity, data[12].humidity, data[17].humidity, data[22].humidity],
                   [data[3].humidity, data[8].humidity, data[13].humidity, data[18].humidity, data[23].humidity],
                   [data[4].humidity, data[9].humidity, data[14].humidity, data[19].humidity, data[24].humidity] ])
    data = np.reshape(data,(5,5)).T

    #Z = np.arange(64).reshape(8,8)
    #Z = Z.reshape((len(X), len(Y)))

    #Global to toggle new data on/off
    isNewData = True

    ###
    
def plot_updater(data=None):
    global X,Y,Z,fig,fsize,levels
    fig.clf()
    fig = plt.figure('humidity_plot', figsize=fsize)
    plt.axis([-1,9,-1,9])
    plt.xlabel(r'$x$ '+ r'$(m)$', fontsize=20)
    plt.ylabel(r'$y$ '+ r'$(m)$',fontsize=20)

    plt.contourf(X, Y, Z, cmap='jet', levels=levels)

    cbar = plt.colorbar()
    cbar.set_label('Relative humidity '+ r'%', size=20) 
    fig.canvas.draw()

def plot_animate(i):###FIX ME
    
    global X,Y,Z,fig,fsize,levels, isNewData
    print 'frame:', i

    fig.clf()
    fig = plt.figure('humidity_plot', figsize=fsize)
    plt.axis([-1,9,-1,9])
    plt.xlabel(r'$x$ '+ r'$(m)$', fontsize=20)
    plt.ylabel(r'$y$ '+ r'$(m)$',fontsize=20)
    
    while not isNewData: 
        time.sleep(.01)    

    cont = plt.contourf(X, Y, Z, cmap='jet', levels=levels)
    cbar = plt.colorbar()
    cbar.set_label('Relative humidity '+ r'%', size=20) 

    fig.canvas.draw()
    
    isNewData = False

    return cont
    
###


#Init main
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("wb_list", wb_list, getData_callback)
    rate      = rospy.Rate(100)
    animate   = rospy.get_param('animate')    
    levels = np.linspace(50,100,1000)

    if animate is True:
        numFrames = rospy.get_param('frames')
        name      = rospy.get_param('animateName')
        ext       = rospy.get_param('ext')

        #Added plot_updater to signature
        ani = animation.FuncAnimation(fig, plot_animate, np.arange(1, 10), repeat=False)
        saveName = name+ext
        
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=15, bitrate=1800)

        #print 'save:', ani.save(saveName, writer=writer)
        ani.save(saveName, writer="ffmpeg")
        #ani.save("video.mp4", writer=writer)
        #plt.show()
    else:              
        plt.ion()
        while not rospy.is_shutdown():            
            if Z is not None:
                plot_updater()
            else:
                print "No data recieved, start/wait for sensor_grid node"
                time.sleep(2)
                rate.sleep()
    
    #import pdb; pdb.set_trace()

