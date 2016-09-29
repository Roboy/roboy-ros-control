## Description ##
roboy-ros-control provides ros control hierarchy for roboy (v2.0) hardware. 
If you have any questions feel free to contact one of the team members from [dynamic_balancing](https://devanthro.atlassian.net/wiki/display/DDB/Development+-+Dynamic+Balancing)

# Installation from launchpad ppa
NOTE: The roboy-ros-control ppa is build for Ubuntu 16.04 (xenial). It is is not available for Ubuntu 14.04 (trusty). ROS jade is not officially supported on Ubuntu xenial, however you can find a full installation of ROS jade [here](https://launchpad.net/~letrend/+archive/ubuntu/ros-jade). Installing roboy-ros-control in the following will automatically install all packages you need, for running the code.
### add the ppas to your apt source list
```
#!bash
sudo add-apt-repository -y ppa:letrend/ros-jade
sudo add-apt-repository -y ppa:letrend/roboy-ros-control
sudo apt-get update
```
### install
```
#!bash
sudo apt-get install roboy-ros-control
```
### In order to use the simulation: symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir ~/.gazebo/models
ln -s /opt/ros/jade/share/roboy_models/legs_with_muscles_simplified ~/.gazebo/models/
ln -s /opt/ros/jade/share/roboy_models/arm ~/.gazebo/models/
ln -s /opt/ros/jade/share/roboy_models/plate_with_muscle ~/.gazebo/models/
```

----

# Run it
## with real roboy
### start the controller_manager
```
#!bash
cd path/to/roboy-ros-control
source devel/setup.bash         # This command with an absolute path could be
                                # added to your ~/.bashrc to ease the use
roslaunch roboy_hardware roboy.launch
```
### initialise the controllers
The format here is based on the msg defined here: https://github.com/Roboy/common_utilities/blob/master/msg/Initialize.msg
You can initialise a number of controllers at the same time.
The values:
- id
- controlmode (defined in [common_utilities/include/CommonDefinitions.h](https://github.com/Roboy/common_utilities/blob/master/include/CommonDefinitions.h#L8))
- resource (aka joint_name in [roboy_controller.yaml](https://github.com/Roboy/roboy_controller/blob/master/config/roboy_controller.yaml))
 need to match the values defined in [roboy_controller.yaml](https://github.com/Roboy/roboy_controller/blob/master/config/roboy_controller.yaml)

**TIP:** Autocomplete is your friend if you use '/' i.e. `rostopic pub / -tab-`
```
#!bash
rostopic pub /roboy/initialize /common_utilities/Initialize "controllers: - {id: 0, controlmode: 0, resource: '', ganglion: 0, motor: 0}"
```

### compile a trajectory for the motors
For **every** motor you need to fill [/common_utilities/Trajectory](https://github.com/Roboy/common_utilities/blob/master/msg/Trajectory.msg) msgs, where samplerate is the frequency defining the time delta between waypoints.
```
#!bash
uint32 id
float32 samplerate
float32[] waypoints
```
The message then should be published on the topic: /roboy/trajectory/**your_joint_name** as defined in i.e. [ForceController.cpp](https://github.com/Roboy/roboy_controller/blob/master/src/ForceController.cpp#L32) (similar for the other controllers)
This will automatically stop the controller of the associated controller.
Do this for all motors you want to set trajectories for.
### start the controllers
Now you can start **all controllers at once**:

A message on topic: `/roboy/steer_record` of type [/common_utilities/steer](https://github.com/Roboy/common_utilities/blob/master/msg/Steer.msg) with the options of this [enum](https://github.com/Roboy/common_utilities/blob/master/include/CommonDefinitions.h#L37):
- 0 (STOP_TRAJECTORY)
- 1 (PLAY_TRAJECTORY)
- 2 (PAUSE_TRAJECTORY)
- 3 (REWIND_TRAJECTORY)

Starts/stops/etc all controllers as defined here: https://github.com/Roboy/roboy_hardware/blob/master/src/roboy.cpp#L275

----

## with walkTrainer
```
#!bash
rviz &
rosrun roboy_simulation walkTrainer
```
In rviz you can add the walking plugin panel for controlling the simulation.

## Usage
List the availale controller types:
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
# Building from source #
The following instructions guide you through the process of building this repo from source.
## Dependencies
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
following the instruction in [this](http://askubuntu.com/questions/466651/how-do-i-use-the-latest-gcc-on-ubuntu) forum:
```
#!bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9
```
### [ROS jade](http://wiki.ros.org/jade/)
For detailed description of installation see [here](http://wiki.ros.org/jade/Installation/Ubuntu). The code also runs with indigo, except for an additional Marker enum in visualization_msgs (ie DELETALL). You can however build the [jade version](https://github.com/ros/common_msgs) from source, but rviz will probably also need to be rebuild with the new message. The following instructions will guide you through the installation for this project. This has been tested on a clean installation of [Ubuntu 14.04](http://releases.ubuntu.com/14.04/). 
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
sudo apt-get install gazebo5 libgazebo5-dev 
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

## clone repos
The project also depends on the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
The repos can be cloned with the folowing commands, where the submodule command attempts to pull the [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) and [common_utilities](https://github.com/Roboy/common_utilities).
```
#!bash
git clone https://github.com/Roboy/roboy-ros-control
cd roboy-ros-control
git submodule update --init --recursive
```
## Build
Please follow the installation instructions for [flexrayusbinterface](https://github.com/Roboy/flexrayusbinterface) before proceeding.
### patching WinTypes.h
Additionally you need to patch two typedefs in WinTypes.h, which comes with the ftd2xx driver, because they are conflicting with the gazebo header FreeImage.h.
```
#!bash
cd path/to/roboy-ros-control/src/myomaster/patches
diff -u /usr/include/WinTypes.h WinTypes.h > WinTypes.diff
sudo patch /usr/include/WinTypes.h < WinTypes.diff
```
NOTE: in case you want to undo the patch run with -R switch:
```
#!bash
cd path/to/roboy-ros-control/src/myomaster/patches
sudo patch -R /usr/include/WinTypes.h < WinTypes.diff
```
### Environmental variables and sourceing
Now this is very important. For both build and especially running the code successfully you will need to define some env variables and source some stuff. Add the following lines to your ~/.bashrc (adjusting the paths to your system):
```
#!bash
source /usr/share/gazebo-5.0/setup.sh
export GAZEBO_MODEL_PATH=/path/to/roboy-ros-control/src/roboy_simulation:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/path/to/roboy-ros-control/devel/lib:$GAZEBO_PLUGIN_PATH
source /opt/ros/jade/setup.bash
source /path/to/roboy-ros-control/devel/setup.bash
```
Then you can build with:
```
#!bash
source ~/.bashrc
cd path/to/roboy-ros-control
catkin_make --pkg common_utilities
source devel/setup.bash
catkin_make
```
### symlink to meshes
For gazebo to find the meshes, create a symlink:
```
#!bash
mkdir ~/.gazebo/models
ln -s path/to/roboy-ros-control/src/roboy_models/legs_with_muscles_simplified ~/.gazebo/models/
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
## Documentation ##
Generate a doxygen documentation using the following command:
```
#!bash
cd path/to/roboy-ros-control
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
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" letrend/roboy-ros-control:devel roslaunch myo_master roboySim.launch
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
cd path/to/roboy-ros-control
docker build -t roboy-ros-control .
```
