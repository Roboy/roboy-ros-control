Ros hierarchy provides ros control hierarchy for roboy (v2.0) hardware via flexray hardware interface and gazebp simulation.
If you have any questions feel free to contact one of the [rosifying team](https://devanthro.atlassian.net/wiki/display/RM/ROSifying+Myorobotics+Development) members.
# Dependencies #
ROS, for detailed description of installation see [here](http://wiki.ros.org/indigo/Installation/Ubuntu)
```
#!bash
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-controller-interface ros-indigo-controller-manager ros-indigo-control-toolbox ros-indigo-gazebo-ros-control
sudo apt-get install libncurses5-dev 
```
project also depends on the [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface) and [common_utilities](https://gitlab.lrz.de/letrend/common_utilities).
The repos can be cloned with the folowing commands, where the submodule commands attempt to pull the [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface) and [common_utilities](https://gitlab.lrz.de/letrend/common_utilities).
This will only be successful if the repo has been shared with you. 
```
#!bash
git clone https://gitlab.lrz.de/rosifyingmyorobotics/ros_hierarchy.git
cd ros_hierarchy
git submodule init
git submodule update
```

# Build #
Please follow the installation instructions for [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface) before proceeding 
Additionally you need to patch two typedefs of the gazebo stuff, because they are incompatible with ftd2xx.h (or rather with the WinTypes.h, ftd2xx.h uses).
```
#!bash
cd path/to/ros_hierarchy/src/myomaster/patches
sudo patch /usr/include/FreeImage.h < FreeImage.h.diff
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
/roboy/emergencyStop
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
You have probably already used the initialize service. Then there is also an emergency stop:
```
#!bash
rosservice call /roboy/emergencyStop [PRESS TAB TWICE]
```
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
types: ['roboy_controller/singleJoint_controller']
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
please follow instructions in [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface), concerning library installation and udev rule.
