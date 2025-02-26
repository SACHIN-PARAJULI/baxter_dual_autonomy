# Import libraries
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import networkx as nx
from matplotlib.animation import FuncAnimation, PillowWriter


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
fig = plt.figure()
ax = Axes3D(fig)
redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='b', marker='o')[0] # For scatter plot
# NOTE: Can't pass empty arrays into 3d version of plot()
line = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='b')[0] # For line plot


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
# fig = plt.figure()
# ax = Axes3D(fig)
redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='c', marker='o')[0] # For scatter plot
# NOTE: Can't pass empty arrays into 3d version of plot()
line = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='c')[0] # For line plot

# AXES PROPERTIES]
# ax.set_xlim3d([limit0, limit1])
ax.set_xlabel('X(t)')
ax.set_ylabel('Y(t)')
ax.set_zlabel('Z(t)')
ax.set_title('Trajectory')


ax.scatter3D(0.77,0.22,0.0, s=50)
ax.scatter3D(0.77,0.02,0.0, s=50)
ax.scatter3D(0.77,-0.18,0.0, s=50)
ax.scatter3D(0.77,-0.38,0.0, s=50)
ax.scatter3D(0.77,-0.58,0.0, s=50)

plt.show()

# https://pythonmatplotlibtips.blogspot.com/2018/11/3d-scatter-plot-animation-funcanimation-python-matplotlib.html

# fn = 'plot_3d_scatter_funcanimation'
# traj_ani.save(fn+'.mp4',writer='ffmpeg',fps=fps)
# traj_ani.save(fn+'.gif',writer='imagemagick',fps=fps)



# file = open("/Users/melanie/Desktop/Baxter_poses.txt","r")
# lines = file.readlines()

# print(len(lines))

# index = 1
# pos_x_array = np.empty(14, dtype=float)
# pos_y_array = np.empty(14, dtype=float)
# pos_z_array = np.empty(14, dtype=float)
# while index <= len(lines):

# 	if not index % 9:
# 		print(index)
# 		print(index / 9)

# 		pos_x_line = lines[index-8].split(": ")
# 		# print(pos_x_line)
# 		pos_x = float(pos_x_line[-1])
# 		print(pos_x)
# 		pos_x_array[int(index/9-1)] = pos_x

# 		pos_y_line = lines[index-7].split(": ")
# 		# print(pos_y_line)
# 		pos_y = float(pos_y_line[-1])
# 		print(pos_y)
# 		pos_y_array[int(index/9-1)] = pos_y

# 		pos_z_line = lines[index-6].split(": ")
# 		# print(pos_z_line)
# 		pos_z = float(pos_z_line[-1])
# 		print(pos_z)
# 		pos_z_array[int(index/9-1)] = pos_z

# 	index = index + 1

# print(pos_x_array)
# print(pos_y_array)
# print(pos_z_array)

# # Creating figure
# fig = plt.figure(figsize = (10, 7))
# ax = plt.axes(projection ="3d")

# # Creating plot
# ax.scatter3D(pos_x_array, pos_y_array, pos_z_array, color = "green")
# ax.plot3D(pos_x_array, pos_y_array, pos_z_array, color = "green")
# # line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='g')[0]
# plt.title("simple 3D scatter plot")

# # Creating the Animation object
# ani = animation.FuncAnimation(
#     fig, update_lines, num_steps, fargs=(walks, lines), interval=100)

# # ANIMATION FUNCTION
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# sct, = ax.plot([], [], [], "o", markersize=2)
# def update(ifrm, xa, ya, za):
#     sct.set_data(xa[ifrm], ya[ifrm])
#     sct.set_3d_properties(za[ifrm])
# ax.set_xlim(0,100)
# ax.set_ylim(0,100)
# ax.set_zlim(0,100)
# num_steps = 30
# ani = animation.FuncAnimation(fig, update, num_steps, fargs=(pos_x_array,pos_y_array,pos_z_array), interval=100)

# # show plot
# plt.show()


# https://www.bragitoff.com/2020/10/3d-trajectory-animated-using-matplotlib-python/

# # Creating dataset
# z = np.random.randint(100, size =(50))
# x = np.random.randint(80, size =(50))
# y = np.random.randint(60, size =(50))

# print('z')
# print(z)
# z
# [29 64 56 31 51 83 57  5 94 17  5 55 45 77 81 94  4 22  8 52 17 93 11  1
#  11 12 43 38 30 70 92 77 13 15 75 42 55 96 57 63 19 10 88 63 16 48 22 25
#  14 69]

# # Creating figure
# fig = plt.figure(figsize = (10, 7))
# ax = plt.axes(projection ="3d")

# # Creating plot
# ax.scatter3D(pos_x_array, pos_y_array, pos_z_array, color = "green")
# plt.title("simple 3D scatter plot")

# # show plot
# plt.show()

# file = open("/Users/melanie/Desktop/traj_legible.txt","r")

# lines = file.readlines()

# x = lines[0].split(", ")

# print(float(x[-1]))


# plan_traj = { }

# Dict = {'Name': 'Geeks', 1: [1, 2, 3, 4]}

# T = ('red', ('green', 'blue'), 'yellow')

# print(T)

# # insert 'purple' between 'green' and 'blue'

# print(T[1])

# T_part = T[1]

# T_part = T_part[:1]+('purple',)+T_part[1:]

# print(T_part)

# T = T[:1]+(T_part,)+T[2:]

# # T = list(T)
# # T.insert(3, 'purple')
# # T = tuple(T)

# print(T)




# References
# https://gist.github.com/neale/e32b1f16a43bfdc0608f45a504df5a84
# https://towardsdatascience.com/animations-with-matplotlib-d96375c5442c
# https://riptutorial.com/matplotlib/example/23558/basic-animation-with-funcanimation

# # ANIMATION FUNCTION
# def func(num, dataSet, line, redDots):
#     # NOTE: there is no .set_data() for 3 dim data...
#     line.set_data(dataSet[0:2, :num])
#     line.set_3d_properties(dataSet[2, :num])
#     redDots.set_data(dataSet[0:2, :num])
#     redDots.set_3d_properties(dataSet[2, :num])
#     return line


# # THE DATA POINTS
# # t = np.arange(0,20,0.2) # This would be the z-axis ('t' means time here)
# # x = np.cos(t)-1
# # y = 1/2*(np.cos(2*t)-1)
# # dataSet = np.array([x, y, t])
# dataSet = np.array([pos_x_array, pos_y_array, pos_z_array])
# numDataPoints = len(pos_x_array)

# # GET SOME MATPLOTLIB OBJECTS
# fig = plt.figure()
# ax = Axes3D(fig)
# redDots = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='r', marker='o')[0] # For scatter plot
# # NOTE: Can't pass empty arrays into 3d version of plot()
# line = plt.plot(dataSet[0], dataSet[1], dataSet[2], c='b')[0] # For line plot

# # AXES PROPERTIES]
# # ax.set_xlim3d([limit0, limit1])
# ax.set_xlabel('X(t)')
# ax.set_ylabel('Y(t)')
# ax.set_zlabel('Z(t)')
# ax.set_title('Trajectory')
# fps = 10 # Frame per sec

# # Creating the Animation object
# traj_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet,line,redDots), interval=1000/fps)
# # line_ani.save(r'Animation.mp4')


# plt.show()

# # https://pythonmatplotlibtips.blogspot.com/2018/11/3d-scatter-plot-animation-funcanimation-python-matplotlib.html

# fn = 'plot_3d_scatter_funcanimation'
# traj_ani.save(fn+'.mp4',writer='ffmpeg',fps=fps)
# # traj_ani.save(fn+'.gif',writer='imagemagick',fps=fps)
