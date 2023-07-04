---
title: "build_custom_robot/building_gazebo_model/joints"
---

# Joint definition
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

The official documentation of how to define a joint in urdf can be found [here](http://wiki.ros.org/urdf/XML/joint). Some important parts of the joint definition are the following:

- `<axis>` defines which axis is used by the joint for movement.
- `<limit>` 
    - `effort` maximum torque/force measured in [Nm]
    - `velocity` maximum speed measured in [m/s] for primatic joints and [rad/s] for revolute joints
    - `lower` minimum allowed joint angle/position measured in [m] for prismatic joints and [rad] for revolute joints.
    - `upper` maximum allowed joint angle/position

## Continuous Joint

Continuous joints rotate around the defined axis (1 degree of freedom).

```xml
<joint type="continuous" name="JOINT_NAME">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100"/>
</joint>
```

## Revolute Joint

Revolute joints also rotate around the defined axis (1 degree of freedom) similar to continuous joints but have a defined minimum and maximum joint angle in [rad].

```xml
<joint type="revolute" name="JOINT_NAME">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-${pi}" upper="${pi}"/>
</joint>
```
## Prismatic Joint

Prismatic joints move along the defined axis (1 degree of freedom) and have a minimum and maximum joint position definedin [m].

```xml
<joint type="prismatic" name="JOINT_NAME">
  <origin xyz="0 0 1.0" rpy="0 0 0"/>
  <child link="CHILD_LINK_NAME"/>
  <parent link="PARENT_LINK_NAME"/>
  <axis xyz="0 0 1" rpy="0 0 0"/>
  <limit effort="100" velocity="100" lower="-1.0" upper="1.0"/>
</joint>
```

## Fixed Joint

Fixed joints are not really joints because all degrees of freedom are blocked.

```xml
<joint name="JOINT_NAME" type="fixed">
  <parent link="PARENT_LINK_NAME"/>
  <child link="CHILD_LINK_NAME" />
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

