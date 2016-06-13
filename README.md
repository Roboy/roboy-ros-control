## Description ##
Ros control provides ros control hierarchy for roboy (v2.0) hardware. 
If you have any questions feel free to contact one of the team members from [rosifying team](https://devanthro.atlassian.net/wiki/display/RM/ROSifying+Myorobotics+Development), or [simulations team](https://devanthro.atlassian.net/wiki/display/SIM/Simulations).
# Dependencies #
git
```
#!bash
sudo apt-get install git
```
ncurses
```
#!bash
sudo apt-get install libncurses5-dev 
```
doxygen[OPTIONAL]
```
#!bash
sudo apt-get install doxygen
```
gcc>4.8(for c++11 support).
remove all gcc related stuff, then following the instruction in [this](http://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu) forum:
```
#!bash
sudo apt-get remove gcc-*
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9
```
[ROS jade](http://wiki.ros.org/jade/), for detailed description of installation see [here](http://wiki.ros.org/jade/Installation/Ubuntu). However, since the simulation part depends on [sdformat v1.5](http://sdformat.org/spec?elem=sdf&ver=1.5), the following instructions will guide you through the installation. This has been tested on a clean installation of [Ubuntu 14.04](http://releases.ubuntu.com/14.04/).
### add the ros ros repo to your source
```
#!bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
```
### install ros desktop and control related stuff
```
#!bash
sudo apt-get install ros-jade-desktop
sudo apt-get install ros-indigo-controller-interface ros-indigo-controller-manager ros-indigo-control-toolbox
```
### install gazebo5 and ros related packages
```
#!bash
sudo apt-get install ros-jade-gazebo-ros-pkgs
sudo apt-get install gazebo5
```
### clone the repos
project also depends on the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
The repos can be cloned with the folowing commands, where the submodule commands attempt to pull the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
```
#!bash
git clone https://github.com/Roboy/ros_control
cd ros_control
git submodule init
git submodule update
```

# Build #
Please follow the installation instructions for [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) before proceeding.
Additionally you need to patch two typedefs of the gazebo stuff, because they are incompatible with ftd2xx.h (or rather with the WinTypes.h, ftd2xx.h uses).
```
#!bash
cd path/to/ros_hierarchy/src/myomaster/patches
diff -u /usr/include/FreeImage.h FreeImage.h > FreeImage.diff
sudo patch /usr/include/FreeImage.h < FreeImage.diff
```
Note: in case you want to undo the patch run with -R switch:
```
#!bash
cd path/to/ros_hierarchy/src/myomaster/patches
sudo patch -R /usr/include/FreeImage.h < FreeImage.diff
```
Then you can build with:
```
#!bash
cd path/to/ros_hierarchy
catkin_init_workspace
rm CMakeLists.txt
catkin_make
```
If the build fails, this is because ros cannot find the headers. You need to source the setup.bash. Use the following commands to add this to your bashrc.
```
#!bash
cd path/to/ros_hierarchy
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
catkin_make
```

# Run it #
```
#!bash

cd path/to/ros_hierarchy
source devel/setup.bash
roscore &
roslaunch myo_master roboy.launch
```
The roboy.launch file loads 24 motors with corresponding joint controllers onto the ros parameter server. 
The commandline will inform you that roboy is not ready. The program is waiting for a motor initialize request.
This will typically come from the [GUI](https://devanthro.atlassian.net/wiki/display/RGIR/Roboy+GUI+in+ROS+Home).

In general, we tried to make the whole system also controllable from the commandline via ROS [services](http://wiki.ros.org/rosservice) 
and ROS [topics](http://wiki.ros.org/rostopic)
```
#!bash
rosservice list
```
This should output (among possibly others) these services:
```
#!bash
/roboy/initialize
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/switch_controller
/controller_manager/unload_controller
```
The following command will request motors to be initialized via the /robo/initialize service:
```
#!bash
rosservice call /roboy/initialize [PRESS TAB TWICE]
```
All services starting with the trailing /roboy give you access to the full functionality of our control hierarchy.
You have probably already used the initialize service. 
Pressing tab twice after the service will tab complete to the valid service message. If this does not work, ROS probably does not 
know about the services yet. 
Try sourceing the setup.bash (bear in mind, that you have to do that for every terminal you open, unless of course you add the 
source command to your ~/.bashrc).:
```
#!bash
source devel/setup.bash
```
Fill out the values as needed (should be self-explanatory).

If you have initialized motors already, the rosservice list command will be augmented by new services for each motor:
```
#!bash
/roboy/trajectory_motor0
/roboy/trajectory_motor1
/roboy/trajectory_motor3
```
Calling the trajectory service will request a trajectory, e.g. like this:
```
#!bash
rosservice call /roboy/trajectory_motor0 [PRESS TAB TWICE]
```

If you call for the controller types:
```
#!bash

rosservice call /controller_manager/list_controller_types
```
This should show our custom controller plugin:
```
#!bash
types: ['roboy_controller/Position_controller']
base_classes: ['controller_interface::ControllerBase']
```

### Status of the controller ###
The status of the controller can be queried via:
```
#!bash
rosservice call /controller_manager/list_controllers
```
This should show the stopped controllers, together with the resources they have been assigned to
```
#!bash
controller: 
  - 
    name: motor0
    state: stopped
    type: roboy_controller/PositionController
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor0']
  - 
    name: motor1
    state: stopped
    type: roboy_controller/VelocityController
    hardware_interface: hardware_interface::VelocityJointInterface
    resources: ['motor1']
  - 
    name: motor3
    state: stopped
    type: roboy_controller/ForceController
    hardware_interface: hardware_interface::EffortJointInterface
    resources: ['motor3']

```

## Test with hardware ##
please follow instructions in [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface), concerning library installation and udev rule.

## Documentation ##
Generate a doxygen documentation using the following command:
```
#!bash
cd path/to/ros_hierarchy
doxygen Doxyfile
```
The documentation is put into the doc folder.
