---
title: "build_custom_robot/building_gazebo_model/links"
---

# Link definition
{: .no_toc }

The official documentation of how to define a link in urdf can be found [here](http://wiki.ros.org/urdf/XML/link).

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## Box

```xml
<xacro:property name="length" value="1.0" />
<xacro:property name="width" value="1.0" />
<xacro:property name="height" value="1.0" />
<xacro:property name="density" value="1" />
<xacro:property name="mass" value="${length*height*width*density}" />
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="${length} ${width} ${height}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <geometry>
      <box size="${length} ${width} ${height}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${mass*(pow(height,2) + pow(width,2))/12}" ixy="0.0" ixz="0.0"
      iyy="${mass*(pow(length,2) + pow(height,2))/12}" iyz="0.0"
      izz="${mass*(pow(length,2) + pow(width,2))/12}"/>
  </inertial>
</link>
```

## Box Macro Definition

```xml
<xacro:macro name="link_box" params="link_name length width height density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${length*height*width*density}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <box size="${length} ${width} ${height}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0.0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <box size="${length} ${width} ${height}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(pow(height,2) + pow(width,2))/12}" ixy="0.0" ixz="0.0"
        iyy="${mass*(pow(length,2) + pow(height,2))/12}" iyz="0.0"
        izz="${mass*(pow(length,2) + pow(width,2))/12}"/>
    </inertial>
  </link>
</xacro:macro>
```

## Box Macro Usage

```xml
<xacro:link_box link_name="LINK_NAME" length="0.1" width="0.1" height="0.1" density="1" mesh_name="nofile">
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_box>
```

## Cylinder

```xml
<xacro:property name="radius" value="1.0" />
<xacro:property name="length" value="1.0" />
<xacro:property name="density" value="1.0" />
<xacro:property name="mass" value="${pi*length*pow(radius,2)*density}" /> 

<link name="LINK_NAME">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="${length}" radius="${radius}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="${length}" radius="${radius}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${mass*(3*pow(radius,2)+pow(length,2))/12}" ixy="0.0" ixz="0.0"
      iyy="${mass*(3*pow(radius,2)+pow(length,2))/12}" iyz="0.0"
      izz="${mass*pow(radius,2)/2}"/>
  </inertial>
</link>
```

## Cylinder Macro Definition

```xml
<xacro:macro name="link_cylinder" params="link_name radius length density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${pi*length*pow(radius,2)*density}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <cylinder length="${length}" radius="${radius}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <cylinder length="${length}" radius="${radius}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(3*pow(radius,2)+pow(length,2))/12}" ixy="0.0" ixz="0.0"
        iyy="${mass*(3*pow(radius,2)+pow(length,2))/12}" iyz="0.0"
        izz="${mass*pow(radius,2)/2}"/>
    </inertial>
  </link>
</xacro:macro>
```

## Cylinder Macro Usage

```xml
<xacro:link_cylinder link_name="LINK_NAME" radius="0.1" length="0.1" density="1" mesh_name="nofile" >
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_cylinder>
```

## Sphere

```xml
<xacro:property name="radius" value="1.0" />
<xacro:property name="density" value="1.0" />
<xacro:property name="mass" value="${(4/3)*pi*pow(radius,3)}" />
<link name="LINK_NAME">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="${radius}"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.0" rpy="0 0 0"/>
    <geometry>
      <sphere radius="${radius}"/>
    </geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="${mass}"/>
    <inertia
      ixx="${2*mass*pow(radius,2)/5}" ixy="0.0" ixz="0.0"
      iyy="${2*mass*pow(radius,2)/5}" iyz="0.0"
      izz="${2*mass*pow(radius,2)/5}"/>
  </inertial>
</link>
```

## Sphere Macro Definition

```xml
<xacro:macro name="link_sphere" params="link_name radius density:=1 mesh_name:='nofile' *origin_vis *origin_col *origin_inertial">
  <xacro:property name="mass" value="${(4/3)*pi*pow(radius,3)}" />
  <link name="${link_name}">
    <visual>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_vis" />
      <geometry>
        <xacro:if value="${mesh_name == 'nofile'}">
          <sphere radius="${radius}"/>
        </xacro:if>
        <xacro:unless value="${mesh_name == 'nofile'}">
          <mesh filename="${mesh_name}" />
        </xacro:unless>
      </geometry>
    </visual>
    <collision>
      <!-- <origin xyz="0 0 0.0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_col" />
      <geometry>
        <sphere radius="${radius}"/>
      </geometry>
    </collision>
    <inertial>
      <!-- <origin xyz="0 0 0" rpy="0 0 0"/> -->
      <xacro:insert_block name="origin_inertial" />
      <mass value="${mass}"/>
      <inertia
        ixx="${2*mass*pow(radius,2)/5}" ixy="0.0" ixz="0.0"
        iyy="${2*mass*pow(radius,2)/5}" iyz="0.0"
        izz="${2*mass*pow(radius,2)/5}"/>
    </inertial>
  </link>
</xacro:macro>
```

## Sphere Macro Usage

```xml
<xacro:link_sphere link_name="LINK_NAME" radius="0.1" density="1" mesh_name="nofile">
  <!-- origin visual -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin collision -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <!-- origin interia -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:link_sphere>
```

## Mesh

```xml
<geometry>
  <mesh filename="package://custom_robot_tutorial/meshes/wheel_mesh.stl" />
</geometry>
```

## Inertial Parameters

An accurate simulation requires accurate dynamic properties of the links. These are defined by the inertial parameters. An official guide on the Gazebo website can be found [here](http://gazebosim.org/tutorials?tut=inertia). An online tool (mesh cleaner) to calculate the inertial parameters for mesh files automatically can be found [here](https://www.hamzamerzic.info/mesh_cleaner/). When using simple shapes, the previous example code for the different links automatically calculates the inertial parameters assuming you use the defined variables. A list of formulas to calculate the inertial parameters for simple shapes can be found [here](https://en.wikipedia.org/wiki/List_of_moments_of_inertia).

**Make sure the center of mass is in the accurate location.** For simple shapes it is in the center of the link, which means, as long as the `<origin>` inside the `<inertial>` tag is the same as the `<origin>` in the `<collision>` tag it should be correct.

