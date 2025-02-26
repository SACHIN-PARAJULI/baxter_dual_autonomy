#!/usr/bin/env python3

# Import libraries
import numpy as np
import matplotlib.pyplot as plt


file = open("/home/rrl/ros_ws_baxter_collision_detection/src/baxter_legible_motion/src/Baxter_poses_legible.txt","r")
# file = open("/Users/melanie/Desktop/Baxter_poses_potentialf.txt","r")
lines = file.readlines()

print(len(lines))

index = 1
pos_x_array = np.empty(int(len(lines)/9), dtype=float)
pos_y_array = np.empty(int(len(lines)/9), dtype=float)
pos_z_array = np.empty(int(len(lines)/9), dtype=float)
while index <= len(lines):

	if not index % 9:
		print(index)
		print(index / 9)

		pos_x_line = lines[index-8].split(": ")
		# print(pos_x_line)
		pos_x = float(pos_x_line[-1])
		print(pos_x)
		pos_x_array[int(index/9-1)] = pos_x

		pos_y_line = lines[index-7].split(": ")
		# print(pos_y_line)
		pos_y = float(pos_y_line[-1])
		print(pos_y)
		pos_y_array[int(index/9-1)] = pos_y

		pos_z_line = lines[index-6].split(": ")
		# print(pos_z_line)
		pos_z = float(pos_z_line[-1])
		print(pos_z)
		pos_z_array[int(index/9-1)] = pos_z

	index = index + 1

print(pos_x_array)
print(pos_y_array)
print(pos_z_array)

dataSet = np.array([pos_x_array, pos_y_array, pos_z_array])
numDataPoints = len(pos_x_array)

# GET SOME MATPLOTLIB OBJECTS
ax = plt.figure().add_subplot(projection='3d')
redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='b', marker='o')[0] # For scatter plot
# NOTE: Can't pass empty arrays into 3d version of plot()
line = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='b', label ="State-of-the-Art")[0] # For line plot


# file = open("/Users/melanie/Desktop/Baxter_poses_legible.txt","r")
file = open("/home/rrl/ros_ws_baxter_collision_detection/src/baxter_legible_motion/src/Baxter_poses_potentialf.txt","r")
lines = file.readlines()

print(len(lines))

index = 1
pos_x_array = np.empty(int(len(lines)/9), dtype=float)
pos_y_array = np.empty(int(len(lines)/9), dtype=float)
pos_z_array = np.empty(int(len(lines)/9), dtype=float)
while index <= len(lines):

	if not index % 9:
		print(index)
		print(index / 9)

		pos_x_line = lines[index-8].split(": ")
		# print(pos_x_line)
		pos_x = float(pos_x_line[-1])
		print(pos_x)
		pos_x_array[int(index/9-1)] = pos_x

		pos_y_line = lines[index-7].split(": ")
		# print(pos_y_line)
		pos_y = float(pos_y_line[-1])
		print(pos_y)
		pos_y_array[int(index/9-1)] = pos_y

		pos_z_line = lines[index-6].split(": ")
		# print(pos_z_line)
		pos_z = float(pos_z_line[-1])
		print(pos_z)
		pos_z_array[int(index/9-1)] = pos_z

	index = index + 1

print(pos_x_array)
print(pos_y_array)
print(pos_z_array)

dataSet = np.array([pos_x_array, pos_y_array, pos_z_array])
numDataPoints = len(pos_x_array)

# GET SOME MATPLOTLIB OBJECTS
redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='c', marker='o')[0] # For scatter plot
# NOTE: Can't pass empty arrays into 3d version of plot()
line = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='c', label ="Ours")[0] # For line plot

# AXES PROPERTIES
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('Z(t)')
ax.set_title('Trajectory')

ax.scatter3D(0.4, -0.5, -0.01, s=50)
ax.scatter3D(0.3, -0.45, -0.01, s=50)
ax.scatter3D(0.35, -0.58, -0.01, s=50)
ax.scatter3D(0.48, -0.43, -0.01, s=50)
ax.scatter3D(0.5, -0.55, -0.01, s=50)
ax.scatter3D(0.8, -0.5, -0.01, s=50)
ax.scatter3D(0.75, -0.4, -0.01, s=50)
ax.scatter3D(0.65, -0.45, -0.01, s=50)
ax.scatter3D(0.75, -0.3, -0.01, s=50)
ax.scatter3D(0.38, -0.25, -0.01, s=50)
ax.scatter3D(0.35, -0.6, -0.01, s=50)
ax.scatter3D(0.45, -0.4, -0.01, s=50)
ax.scatter3D(0.7, -0.7, -0.01, s=50)
ax.scatter3D(0.5, -0.6, -0.01, s=50)
ax.scatter3D(0.6, -0.33, -0.01, s=50)
ax.scatter3D(0.8, -0.4, -0.01, s=50)
ax.scatter3D(0.3, -0.6, -0.01, s=50)
ax.scatter3D(0.7, -0.5, -0.01, s=50)
ax.scatter3D(0.55, -0.55, -0.01, s=50)
ax.scatter3D(0.62, -0.38, -0.01, s=50)

ax.view_init(elev=20., azim=-35, roll=0)

# Function add a legend
plt.legend(bbox_to_anchor =(0.75, 0.0), ncol = 2)

plt.show()
