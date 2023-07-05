---
title: "build_custom_robot/building_gazebo_model/exercise"
---
# Exercise

Download the ROS package used for the exercise from this link. After unzipping it, copy it to the VM in catkin_ws/src/. Run the following commands in a terminal window:

```bash
cd ~/catkin_ws/
catkin_make
```

Copy the following code into the file “mobile_manipulator_robot.urdf.xacro” located in the “urdf” folder of the ROS package:

```xml
<?xml version='1.0'?>

<robot name="mobile_manipulator_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find custom_robot_tutorial)/urdf/common_macros.xacro" />

  <gazebo>
    <static>true</static>
  </gazebo>

<!--############################### -->
<!-- MOBILE PLATFORM -->
<!--############################### -->
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="mobile_body_link" />
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
  </joint>


  <!-- MOBILE BASE -->
  <!-- ==================================== -->
  <xacro:link_box link_name="mobile_body_link" length="0.65" width="0.4" height="0.2" mesh_name="package://custom_robot_tutorial/meshes/base_mesh.stl">
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_box>


  <!-- FRONT LEFT WHEEL -->
  <!-- ==================================== -->
  <joint type="continuous" name="wheel_front_left_joint">
    <origin xyz="0.200 0.255 -0.050" rpy="-${pi/2} 0 0"/>
    <child link="wheel_front_left_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100"/>
  </joint>

  <xacro:link_cylinder link_name="wheel_front_left_link" radius="0.1" length="0.1" density="1" mesh_name="package://custom_robot_tutorial/meshes/wheel_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_cylinder>

  <!-- FRONT RIGHT WHEEL -->
  <!-- ==================================== -->
  <joint type="continuous" name="wheel_front_right_joint">
    <origin xyz="0.200 -0.255 -0.050" rpy="-${pi/2} 0 0"/>
    <child link="wheel_front_right_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100"/>
  </joint>

  <xacro:link_cylinder link_name="wheel_front_right_link" radius="0.1" length="0.1" density="1" mesh_name="package://custom_robot_tutorial/meshes/wheel_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:link_cylinder>

</robot>
```

To start the Gazebo world use the following command in the terminal:

```bash
roslaunch custom_robot_tutorial mobile_manipulator.launch
```

To spawn the robot in the existing Gazebo world use the following terminal command:

```bash
roslaunch custom_robot_tutorial robot_spawn.launch
```

Start adding the missing 3 wheels to the robot model by modifying the “mobile_manipulator_robot.urdf.xacro” file. Where to place the wheels can be deducted from the following mechanical drawings:

![alt]({{ site.url }}{{ site.baseurl }}/assets/images/build_custom_robot/mobile_robot_plan.png)
![alt]({{ site.url }}{{ site.baseurl }}/assets/images/build_custom_robot/robot_arm_plan.png)

Once you finished with adding all the wheels to the mobile platform, copy the following code into the `mobile_manipulator_robot.urdf.xacro` file just before the `</robot>` tag at the end of the file:

```xml
<!--############################### -->
<!-- ROBOTIC ARM -->
<!--############################### -->

  <!-- ARM BASE -->
  <!-- ==================================== -->
  <joint type="revolute" name="arm_base_joint">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <child link="arm_base_link"/>
    <parent link="mobile_body_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100" lower="-${pi}" upper="${pi}"/>
  </joint>

  <xacro:link_cylinder link_name="arm_base_link" radius="${0.135/2}" length="0.2" density="1" mesh_name="package://custom_robot_tutorial/meshes/arm_base_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </xacro:link_cylinder>

  <!-- LINK 1 -->
  <!-- ==================================== -->
  <joint type="revolute" name="link_1_joint">
    <origin xyz="0 0 0.2" rpy="-${pi/2} 0 0"/>
    <child link="link_1_link"/>
    <parent link="arm_base_link"/>
    <axis xyz="0 0 1" rpy="0 0 0"/>
    <limit effort="100" velocity="100" lower="-${pi/2}" upper="${pi/2}"/>
  </joint>

  <xacro:link_cylinder link_name="link_1_link" radius="${0.075/2}" length="0.385" density="1" mesh_name="package://custom_robot_tutorial/meshes/link_1_mesh.stl" >
    <!-- origin visual -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
    <!-- origin collision -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
    <!-- origin interia -->
    <origin xyz="0 -${0.385/2} 0" rpy="${pi/2} 0 0"/>
  </xacro:link_cylinder>
```