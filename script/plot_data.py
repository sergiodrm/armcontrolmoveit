#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from matplotlib import pyplot as plt
import matplotlib.animation
import numpy as np
import time

chiv = False
t = []
pos = [[], [], [], [], [], [], []]
plt.ion()
fig = plt.figure("Representacion de datos")
ax = fig.add_subplot(111)
line = []
ax.set_title("Joint position")
ax.set_xlabel('Tiempo [s]')
ax.set_ylabel('Posicion [rad]')
ax.set_ylim(-2*np.pi, 2*np.pi)
for i in range(7):
    line.append(ax.plot(t, pos[i], label='j'+str(i+1)))
ax.legend(loc='upper left')
ax.grid("on")
fig.canvas.draw()


def update(data):
    global chiv
    if len(t) > 100:
        t.remove(t[0])
        t.append(data.header.seq)
        
        for i in range(len(line)):
            pos[i].remove(pos[i][0])
            pos[i].append(data.position[i])
    else:
        t.append(data.header.seq)
        for i in range(len(line)):
            pos[i].append(data.position[i])
    chiv = True
    time.sleep(0.05)


def redraw():
    global chiv
    while not rospy.is_shutdown():
        if chiv:
            for i in range(len(line)):
                line[i][0].set_data(t, pos[i])
                ax.set_xlim(min(t), max(t)+1)
            fig.canvas.draw()
            chiv = False
            
        time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('read_topic')
    rospy.Subscriber("/joint_states", JointState, update)
    fig.canvas.flush_events()
    redraw()
