# ros_autonomous_boat

meter a instalação do catkin_virtualenv
meter a compilação externa do cv_bridge
sque meter instruçe sbacanas e meter a readme bonita
ya

install joystick node and otehr stuff:
`
sudo apt-get install ros-melodic-teleop-twist-joy ros-melodic-joy ros-melodic-joy-teleop
`
you can see if joystick is connected succsessfully by running:
`
ls /dev/input

sudo jstest /dev/input/js0
`
then run the node:
`
rosrun joy joy_node dev:=/dev/int/js0
`
