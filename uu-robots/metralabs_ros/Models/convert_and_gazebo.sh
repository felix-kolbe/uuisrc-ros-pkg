#!/bin/bash

name=scitos_haw_schunk

echo killing $name
rosrun gazebo gazebobody kill $name &
#deprecated rosrun gazebo_tools gazebo_model kill $name

sh convert_xacro.sh

echo spawning $name
rosrun gazebo spawn_model -urdf -file $(rospack find metralabs_ros)/Models/${name}.urdf -model $name

