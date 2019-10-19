## Project IFAC World Congress 2020

Repository dedicated to the implementation of Adaptive Artificial Potential Fields by Zhang (2017) for the UR5 robotic manipulator using ROS. A new method for controlling UR5 orientation using Artificial Potential Field is proposed. The full project description will be available shortly after acceptance of the publication in IFAC World Congress (2020)

## Author Information
Authors: Caio Cristiano Barros Viturino, Ubiratan de Melo Pinto Junior, André Gustavo Scolari Conceição and Leizer Schnitman

C. C. B. Viturino, U. M. P. Junior and A. G. S. Conceiçãoo are with the LaR - Robotics Laboratory, Department of Electrical and Computer Engineering, Federal University of Bahia, Salvador, Brazil.
E-mails: engcaiobarros@gmail.com, eng.ele.ubiratan@gmail.com, andre.gustavo@ufba.br.

L. Schnitman is with Department of Chemical Engineering, Federal University of Bahia, Salvador, Brazil. E-mail: leizer@ufba.br.

## Usage
### To use UR5 with RVIZ + gazebo

To bring up the simulated robot in Gazebo, run:

`roslaunch ur_gazebo ur5.launch`

For setting up the MoveIt! nodes to allow motion planning run:

`roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true`

In order to start RViz with a configuration including the MoveIt! Motion Planning plugin run:

`roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

Run the node corresponding to the file UR5_CPA_Gazebo.py

`rosrun custom_codes UR5_CPA_Gazebo.py`

If you want to see the path beeing publish to RVIZ please run publish_trajectory.py and run UR5_CPA_Gazebo with --plot parameter

`rosrun custom_codes publish_trajectory.py`

`rosrun custom_codes UR5_CPA_Gazebo.py --plot`

## Required packages

See custom_codes/requirements.txt for more info

## Changes in universal_robot pkg

Substitute custom_codes/moveit.rviz in universal_robot/ur5_moveit_config/moveit.rviz

If TF lookupTransform is used, it is necessary to create a new frame corresponding to the end effector. To do this, add the following lines to the ur5.urdf.xacro file:

```html
<xacro:property name="tool0_offset" value="0.15" />

<link name="${prefix}grasping_link"/>
<joint name="${prefix}ur5_joint_grasping_frame" type="fixed">
  <origin xyz="0 0 ${tool0_offset}" rpy="0 0 0"/>
  <parent link="${prefix}tool0"/>
  <child link="${prefix}grasping_link"/>
</joint>
```

Add these lines to ur_gazebo/ur5gripper_controllers.yaml

```yaml
gripper:
  type: position_controllers/JointTrajectoryController
  joints:
     - robotiq_85_left_knuckle_joint
  constraints:
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
      simple_gripper_robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
  stop_trajectory_duration: 0.5
  state_publish_rate:  25
  action_monitor_rate: 10
```

Add these lines to ur_gazebo/ur5.launch

```html
<rosparam file="$(find ur_gazebo)/controller/ur5gripper_controllers.yaml" command="load"/>
<node name="gripper_controller_spawner" pkg="controller_manager" type="spawner" args="gripper --shutdown-timeout 0.5"/>
```

Change ur5_moveit_config/config/controllers.yaml according to the following:

```yaml
controller_list:
#-------------for gazebo UR5-------------------------------
  - name: "arm_controller"
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

#-------------for gazebo gripper---------------------------
  - name: "gripper"
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - robotiq_85_left_knuckle_joint
```

Change ur5_moveit_config/config/fake_controllers.yaml according to the following:

```yaml
controller_list:
  - name: fake_manipulator_controller
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
  - name: fake_gripper_controller
    joints:
      - robotiq_85_left_knuckle_joint # modified -> original []
initial:
  - group: manipulator
    pose: up

```

Change ur5_moveit_config/ur5.srdf according to the following:

create a group called manipulator according to the code:

```html
<group name="manipulator">
    <chain base_link="base_link" tip_link="robotiq_coupler" />
</group>
```
create a group called only_manipulator according to the code:

```html
<group name="only_manipulator">
    <chain base_link="base_link" tip_link="wrist_3_link" />
</group>
```
create a group called gripper according to the code:

```html
<group name="gripper">
    <chain base_link="robotiq_85_base_link" tip_link="robotiq_85_left_knuckle_link" />
</group>
```

Link the gripper and the manipulator:

```html
<end_effector name="gripper" parent_link="robotiq_coupler" group="gripper" parent_group="manipulator"/>
```

Configure the passive joint as follows:

```html
<!--PASSIVE JOINT: Purpose: this element is used to mark joints that are not actuated-->
<passive_joint name="robotiq_85_left_inner_knuckle_joint" />
<passive_joint name="robotiq_85_left_finger_tip_joint" />
<passive_joint name="robotiq_85_right_inner_knuckle_joint" />
<passive_joint name="robotiq_85_right_finger_tip_joint" />
<passive_joint name="robotiq_85_right_knuckle_joint" />
```
Disable collision detection:

```html
<disable_collisions link1="ee_link" link2="robotiq_85_base_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="ee_link" link2="robotiq_85_right_knuckle_link" reason="Never" />

<disable_collisions link1="wrist_1_link" link2="robotiq_85_base_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_1_link" link2="robotiq_85_right_knuckle_link" reason="Never" />

<disable_collisions link1="wrist_2_link" link2="robotiq_85_base_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_2_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_base_link" reason="Adjacent" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="robotiq_coupler" reason="Never" />

<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_inner_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_inner_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_left_finger_tip_link" reason="Default" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_left_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_left_inner_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_inner_knuckle_link" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_inner_knuckle_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_inner_knuckle_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_inner_knuckle_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_inner_knuckle_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_knuckle_link" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_knuckle_link" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_knuckle_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_left_knuckle_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_right_finger_link" link2="robotiq_85_right_finger_tip_link" reason="Default" />
<disable_collisions link1="robotiq_85_right_finger_link" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_right_finger_link" link2="robotiq_85_right_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_right_finger_tip_link" link2="robotiq_85_right_inner_knuckle_link" reason="Adjacent" />
<disable_collisions link1="robotiq_85_right_finger_tip_link" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_85_right_inner_knuckle_link" link2="robotiq_85_right_knuckle_link" reason="Never" />

<disable_collisions link1="robotiq_85_base_link" link2="tool0" reason="Never" />
<disable_collisions link1="wrist_3_link" link2="tool0" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="tool0" reason="Adjacent" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_base_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_left_finger_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_left_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_left_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_left_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_right_finger_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_right_finger_tip_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_right_inner_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="robotiq_85_right_knuckle_link" reason="Never" />
<disable_collisions link1="robotiq_coupler" link2="ee_link" reason="Never" />
```

## How to connect to the real UR5 robot

Firstly check the machine IP. The IP configured on the robot must have the last digit different.

`ifconfig`

Disable firewall

`sudo ufw disable`

Set up a static IP on UR5 according to the following figure

![IP Config UR5](https://github.com/caiobarrosv/Projeto_ICAR_2019/blob/master/imgs/config.jpg)

Set up a connection on Ubuntu according to the following figure

![IP Config Ubuntu](https://github.com/caiobarrosv/Projeto_ICAR_2019/blob/master/imgs/config_ethernet2.jpg)

Start ROS with RViz + Gazebo

Bring up - The configured ip must be the same as the robot

`roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=169.254.113.30`
