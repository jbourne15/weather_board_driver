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
fig = plt.figure('humidity_plot', figsize=(10,10))
ax = fig.add_subplot(111, projection='3d')
line = 0

X = np.arange(5)
Y = np.arange(5)
X, Y = np.meshgrid(X, Y)
        
# Set up formatting for the movie files
#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

###

def plotCallback(data):
    data = data.wb_list
    global X, Y, ax, fig, Writer, writer, line

    #DON'T TOUCH PAST HERE!!!------------------------------------------
    #Except for a few lines for the lines.
    ###        
    ani = animation.FuncAnimation(fig, surface_updater, fargs=(data, line),
                                       interval=50, blit=False)
    ani.save('humidity.mp4', dpi=250)
    ###

def plotCallback2(data):
    global X, Y, ax, fig
    data = data.wb_list
    Z = np.array([ [data[0].humidity, data[1].humidity, data[2].humidity, data[3].humidity, data[4].humidity],
                   [data[5].humidity, data[6].humidity, data[7].humidity, data[8].humidity, data[9].humidity],
                   [data[10].humidity, data[11].humidity, data[12].humidity, data[13].humidity, data[14].humidity],
                   [data[15].humidity, data[16].humidity, data[17].humidity, data[18].humidity, data[19].humidity],
                   [data[20].humidity, data[21].humidity, data[22].humidity, data[23].humidity, data[24].humidity] ])    
    fig = plt.figure('humidity_plot', figsize=(10,10))
    fig.clf()
    
    line = ax.plot_surface(X, Y, Z)#, cmap=cm.coolwarm)
    fig.canvas.draw()
    #time.sleep(.5)
    
    ###
    
def surface_updater(i, data, line):
    
    Z = np.array([ [data[0].humidity, data[1].humidity, data[2].humidity, data[3].humidity, data[4].humidity],
                 [data[5].humidity, data[6].humidity, data[7].humidity, data[8].humidity, data[9].humidity],
                 [data[10].humidity, data[11].humidity, data[12].humidity, data[13].humidity, data[14].humidity],
                 [data[15].humidity, data[16].humidity, data[17].humidity, data[18].humidity, data[19].humidity],
                 [data[20].humidity, data[21].humidity, data[22].humidity, data[23].humidity, data[24].humidity] ])
         
    ax.clear()

    line = ax.plot_surface(X, Y, Z)#, cmap=cm.coolwarm)
    plt.show()
    return line,
    
#Init main
if __name__ == '__main__':
     
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("wb_list", wb_list, plotCallback2)
    rospy.spin()

