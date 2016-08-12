## Description ##
Ros control provides ros control hierarchy for roboy (v2.0) hardware. 
If you have any questions feel free to contact one of the team members from [rosifying team](https://devanthro.atlassian.net/wiki/display/RM/ROSifying+Myorobotics+Development), or [simulations team](https://devanthro.atlassian.net/wiki/display/SIM/Simulations).
# Dependencies #
### git
```
#!bash
sudo apt-get install git
```
### ncurses
```
#!bash
sudo apt-get install libncurses5-dev 
```
### doxygen[OPTIONAL]
```
#!bash
sudo apt-get install doxygen
```
### gcc>4.8(for c++11 support).
remove all gcc related stuff, then following the instruction in [this](http://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu) forum:
```
#!bash
sudo apt-get remove gcc-*
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9
```
### [ROS jade](http://wiki.ros.org/jade/)
For detailed description of installation see [here](http://wiki.ros.org/jade/Installation/Ubuntu). However, the following instructions will guide you through the installation for this project. This has been tested on a clean installation of [Ubuntu 14.04](http://releases.ubuntu.com/14.04/).
```
#!bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
```
#### install ros desktop and control related stuff
```
#!bash
sudo apt-get install ros-jade-desktop
sudo apt-get install ros-jade-controller-interface ros-jade-controller-manager ros-jade-control-toolbox ros-jade-transmission-interface ros-jade-joint-limits-interface
```
#### install gazebo5 and gazebo-ros-pkgs
```
#!bash
sudo apt-get install gazebo5
sudo apt-get install ros-jade-gazebo-ros-pkgs
```
You should try to run gazebo now, to make sure its working. 
```
#!bash
source /usr/share/gazebo-5.0/setup.sh
gazebo --verbose
```
If you seen an output like, 'waiting for namespace'...'giving up'. Gazebo hasn't been able to download the models. You will need to do this manually. Go to the osrf [bitbucket](https://bitbucket.org/osrf/gazebo_models/downloads), click download repository. Then unzip and move to gazebo models path:
```
#!bash
cd /path/to/osrf-gazebo_models-*.zip
unzip osrf-gazebo_models-*.zip -d gazebo_models
mv gazebo_models/* ~/.gazebo/models
```
Now we need to tell gazebo where to find these models. This can be done by setting the GAZEBO_MODEL_PATH env variable. Add the following line to your ~/.bashrc:
```
#!bash
export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH
```
If you run gazebo now it should pop up without complaints and show an empty world.
# Installation
project also depends on the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
## clone repos
The repos can be cloned with the folowing commands, where the submodule commands attempt to pull the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
```
#!bash
git clone https://github.com/Roboy/ros_control
cd ros_control
git submodule init
git submodule update
```

## Build
Please follow the installation instructions for [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) before proceeding.
Additionally you need to patch two typedefs in WinTypes.h, which comes with the ftd2xx driver, because they are conflicting with the gazebo header FreeImage.h.
```
#!bash
cd path/to/ros_control/src/myomaster/patches
diff -u /usr/include/WinTypes.h WinTypes.h > WinTypes.diff
sudo patch /usr/include/WinTypes.h < WinTypes.diff
```
NOTE: in case you want to undo the patch run with -R switch:
```
#!bash
cd path/to/ros_control/src/myomaster/patches
sudo patch -R /usr/include/WinTypes.h < WinTypes.diff
```
### installation of gazebo qt overlay
Add the following lines in the file ~/.gazebo/gui.ini:
```
#!bash
[overlay_plugins]
filenames=libGazeboRoboyOverlay.so
```
### Environmental variables and sourceing
Now this is very important. For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjusting the paths to your system):
```
#!bash
source /usr/share/gazebo-5.0/setup.sh
export GAZEBO_MODEL_PATH=/path/to/ros_control/src/roboy_simulation:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/ros_control/devel/lib:$GAZEBO_PLUGIN_PATH
source /opt/ros/jade/setup.bash
source /path/to/ros_control/devel/setup.bash
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/ros_control
catkin_make
```
#### If the build fails throwing an error like 'Could not find a package configuration file provided by "gazebo_ros_control"',
this is because for some mysterious reason gazebo_ros_pkgs installation is degenrate. But that won't stop us. We will build it from source. 
```
#!bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs
cd gazebo_ros_pkgs
git checkout jade-devel
cd ~/ros_ws
sudo -s
source /opt/ros/jade/setup.bash
catkin_make_isolated --install --install-space /opt/ros/jade/ -DCMAKE_BUILD_TYPE=Release
exit
```
#### If the build fails, complaining about missing headers,
this is probably because ros cannot find the headers it just created. You need to source the devel/setup.bash:
```
#!bash
source devel/setup.bash
catkin_make
```
# Run it
## with real roboy
```
#!bash
cd path/to/ros_control
source devel/setup.bash
roslaunch myo_master roboy.launch
```
## with simulated roboy
```
#!bash

roslaunch myo_master roboySim.launch
```
## Usage
Calling for controller types:
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
## Documentation ##
Generate a doxygen documentation using the following command:
```
#!bash
cd path/to/ros_control
doxygen Doxyfile
```
The documentation is put into the doc folder.

# docker 
This repo is build into a docker image. Please follow the [docker install instructions](https://docs.docker.com/engine/installation/) for your system.
## usage
You can run eg the simulation with the following commands:
```
#!bash
xhost +local:root
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" letrend/ros_control:devel roslaunch myo_master roboySim.launch
```
The xhost command enables GUI rendering, please check this [page](http://wiki.ros.org/docker/Tutorials/GUI) for alternatives.
The docker run command downloads the image and runs the following commands. Once you are done, disable xhost with:
```
#!bash
xhost -local:root
```
## build
You can also build your own docker image, using the Dockerfile in the repo with the following command:
```
#!bash
cd path/to/ros_control
docker build -t ros_control .
```
