#!/usr/bin/env python

import sys

import numpy as np
import scipy as sp

import skimage.io as io
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import matplotlib.colors as mcol
from mpl_toolkits import mplot3d

logFile = sys.argv[1]

labelSize = 10
axisSize = 10
legendSize = 10

filename = '/media/xisco/Data/Documents/FeinaUIB-SRV/projectes/ROBINS/experiments/genovaJune19/UWB/manual/expUWB/' + logFile + '_POZYX.txt'
# filename2 = '/media/xisco/Data/Documents/FeinaUIB-SRV/projectes/ROBINS/experiments/Munkebo Mar 2019/UWB/manual/expUWB/ICPexp' + logFile + '_final.txt'
filename2 = '/media/xisco/Data/Documents/FeinaUIB-SRV/projectes/ROBINS/experiments/genovaJune19/UWB/manual/expUWB/' + logFile + '_ICP.txt'

data = np.loadtxt(filename,delimiter=',', skiprows=1)
data2 = np.loadtxt(filename2,delimiter=',', skiprows=1)

fig = plt.subplot()
# hold on

# rectangle
sizeX = 16
sizeY = 4
height = 2
# plt.plot([0,sizeX,sizeX,0,0],[0,0,sizeY,sizeY,0], 'k', LineWidth = 2, label='16x4 m rectangle')
# plot3([0,sizeX,sizeX,0,0],[0,0,sizeY,sizeY,0],ones(5)*height, 'k', LineWidth = 2)

# plot3(data[:,1],data[:,2],data[:,4], 'b', LineWidth = 2)
# plot3(data[1,1],data[1,2],data[1,4], 'og', LineWidth = 2)
# plot3(data[-1,1],data[-1,2],data[-1,4], 'or', LineWidth = 2)
plt.plot(data[:,1],data[:,2], 'b', LineWidth = 2, label='Original output')
plt.plot(data[1,1],data[1,2], 'og', LineWidth = 2)#, label='Initial points')
plt.plot(data[-1,1],data[-1,2], 'or', LineWidth = 2)#, label='Final points')

# plot3(data2[:,1],data2[:,2],data2[:,4], 'red',LineWidth = 2)
# plot3(data2[1,1],data2[1,2],data2[1,4], 'og', LineWidth = 2)
# plot3(data2[-1,1],data2[-1,2],data2[-1,4], 'or', LineWidth = 2)
plt.plot(data2[:,1],data2[:,2], 'red',LineWidth = 2, label='Our method')
plt.plot(data2[1,1],data2[1,2], 'og', LineWidth = 2, label='Initial points')
plt.plot(data2[-1,1],data2[-1,2], 'or', LineWidth = 2,  label='Final points')

plt.legend(loc=2, fontsize=legendSize)
plt.xlabel('X (m)', FontSize = labelSize)
plt.ylabel('Y (m)', FontSize = labelSize)

# Set the tick labels font
for label in (fig.get_xticklabels() + fig.get_yticklabels()):
    # label.set_fontname('Arial')
    label.set_fontsize(axisSize)

plt.grid()
plt.axis('equal')
# axis equal
# # h = gca
# # h.YTick = -0.8:0.2:0.8
# # h.XTick = 0:5:50

# # print('-dpng', ['/home/xisco/ROS/lunar/src/robins/pozyx_drivers/scripts/','sqPozyx.png'])

# plt.savefig(logFile+'.pdf')

plt.figure()
fig = plt.subplot()

ax = plt.axes(projection='3d')
ax.plot3D(data[:,1],data[:,2],data[:,3],'b')
ax.plot3D(data2[:,1],data2[:,2],data2[:,3],'red')
ax.set_xlabel('X(m)')
ax.set_ylabel('Y(m)')
ax.set_zlabel('Z(m)')

plt.axis('equal')

io.show()
