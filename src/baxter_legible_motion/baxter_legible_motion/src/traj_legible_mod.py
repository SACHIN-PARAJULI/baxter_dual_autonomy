# Import libraries
import numpy as np
import matplotlib.pyplot as plt


file = open("/home/rrl/ros_ws_baxter_collision_detection/src/pick_and_place_baxter_moveit/src/Baxter_poses_legible.txt","r")
# file = open("/Users/melanie/Desktop/Baxter_poses_potentialf.txt","r")
lines = file.readlines()

print(len(lines))

index = 1
pos_x_array = np.empty(int(len(lines)/9), dtype=float)
pos_y_array = np.empty(int(len(lines)/9), dtype=float)
pos_z_array = np.empty(int(len(lines)/9), dtype=float)
# pos_x_array = np.empty(14, dtype=float)
# pos_y_array = np.empty(14, dtype=float)
# pos_z_array = np.empty(14, dtype=float)
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
file = open("/home/rrl/ros_ws_baxter_collision_detection/src/pick_and_place_baxter_moveit/src/Baxter_poses_potentialf.txt","r")
lines = file.readlines()

print(len(lines))

index = 1
pos_x_array = np.empty(int(len(lines)/9), dtype=float)
pos_y_array = np.empty(int(len(lines)/9), dtype=float)
pos_z_array = np.empty(int(len(lines)/9), dtype=float)
# pos_x_array = np.empty(14, dtype=float)
# pos_y_array = np.empty(14, dtype=float)
# pos_z_array = np.empty(14, dtype=float)
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


ax.scatter3D(0.77,0.22,-0.03, s=50)
ax.scatter3D(0.77,0.02,-0.03, s=50)
ax.scatter3D(0.77,-0.18,-0.03, s=50)
ax.scatter3D(0.77,-0.38,-0.03, s=50)
ax.scatter3D(0.77,-0.58,-0.03, s=50)

ax.view_init(elev=20., azim=-35, roll=0)

# # Function to plot
# plt.plot(y1, label ="y = x")
# plt.plot(y2, label ="y = 3x")

# Function add a legend
# plt.legend(bbox_to_anchor =(0.75, 1.15), ncol = 2)
plt.legend(bbox_to_anchor =(0.75, 0.0), ncol = 2)

plt.show()
