# baxter_dual_autonomy
## Getting This Repository and File Structure
### Cloning and First Steps
First clone this repository, AS your ros workspace. 

`git clone https://github.com/SACHIN-PARAJULI/baxter_dual_autonomy.git`

Next, buld your workspace with catkin

`catkin_make`

See dependencies section for installing anything necessary.

The file structure should be laid out with some txt files, and folders in the top level of the repo
- devel
- src
- build
- txt's, the README
- **manuel.sh**

## Startup Process

Once inside the repo, with the initial catkin_make command run, you can source and connect to your baxter (Xavier in our case)

```
source devel/setup.bash
./manuel.sh
```

Finally, enable the robot

`rosrun baxter_tools enable_robot.py`

If you desire the arms to be in a better position, use the untuck command

`rosrun baxter_tools tuck_arms.py -u`

## Pick and Place Steps

1. Run the action server first `rosrun baxter_interface joint_trajectory_action_server.py`

2. Run the rviz gripper config next `roslaunch baxter_moveit_config baxter_grippers.launch`

3. Run the pick and place command, providing the txt file to use as an argument `rosrun moveit_check test_moveit.py example.txt`

### Setting new positions for pick and place

1. Use the rostopic for the right arm's endpoint `rostopic echo /robot/limb/right/endpoint_state`

  Further, you should use the pose: position and pose: orientation values. These 7, (3 position, 4 orientation) will give you the values for assigning to the
  position txt file.

2. Format the txt file as such

  ```
    x, y, z
    x, y, z, w
  ```
### Setting NEW NEW Postions With Joints, Over Inferior Cartesian Coords
`rostopic echo /robot/joint_states -n 1`

Please note, the order in this is possibly incorrect, this is the order to write in values for the moveit_script. 

> right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2

While the rostopic will be slightly out of this order.

### Stopping The Pick and Place

Upon the script sticking on output `stopping`, you may `ctrl-c` to kill the process. 

## To Be Continued


## Stopping The Robot

When finished, the following can be used to stop Baxter

`rosrun baxter_tools tuck_arms.py -t`

This command also disables the robot, no need for an extra command


For special lab purposes
0.568515756701146, -0.4082130956181553, -0.2537000900711289
0.9972845588207063, -0.05058431548816209, 0.04447726309507141, 0.029774298177371462
0.7973654744629252, -0.4375103647269141, -0.24892058972158837
0.9944115649105097, -0.09232845257316259, 0.04243894095222418, 0.02863621323656632
0.5686358715227012, -0.3551675812417457, -0.25893919084420547
0.9980910984889041, -0.06018569180155758, -0.0023439649694773496, 0.013650913808341892
0.8047250504788628, -0.3629187735597911, -0.2516354221432035
0.995778396806392, -0.07503372623079681, 0.036454984616253495, 0.03829306044497367
0.717428244632526, -0.0419273660170897, -0.25708857406942365
0.9980680958106287, -0.060490100143100425, 0.012428612969645573, 0.006823011745125251
0.8156471826127519, -0.28812807545548796, -0.2540615342752611
0.9972847238096875, -0.06455735417337828, 0.03056028734529222, 0.017933112260329523
0.5764720506647634, -0.3073123460207638, -0.25695710774669567
0.9972698043367731, -0.06577431185026628, 0.01447129394475257, 0.030286282543136575
0.8243427322893615, -0.21453229633914964, -0.2540888476296819
0.9974587505599422, -0.05874892522423523, 0.002839847144545027, 0.040206218232908125

