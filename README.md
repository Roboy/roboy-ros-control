This illustrates ros hierarchy applied to control of a myode muscle via flexray hardware interface
# Dependencies #

```
#!bash
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-controller-interface ros-indigo-controller-manager ros-indigo-control-toolbox ros-indigo-gazebo-ros-control
```
project also depends on the [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface) and [common_utilities](https://gitlab.lrz.de/letrend/common_utilities).
The repos can be cloned with the folowing commands, where the submodule commands attempt to pull the [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface) and [common_utilities](https://gitlab.lrz.de/letrend/common_utilities).
This will only be successful if the repo has been shared with you. Please contact one of the [rosifying team](https://devanthro.atlassian.net/wiki/display/RM/ROSifying+Myorobotics+Development) members to grant access to you.
```
#!bash
git clone https://gitlab.lrz.de/rosifyingmyorobotics/ros_hierarchy.git
cd ros_hierarchy
git submodule init
git submodule update
```

# Build #

```
#!bash

cd path/to/ros_hierarchy
catkin_init_workspace
rm CMakeLists.txt
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
This will typically come from the [GUI](https://devanthro.atlassian.net/wiki/display/RGIR/Roboy+GUI+in+ROS+Home), but can also be 
done from commandline. The followin command will request motors 0, 1 and 3 to be initialized:
```
#!bash
rosservice call /roboy/initialize '['0', '1', '3']'
```
In general, we tried to make the whole system also controllable from the commandline via ROS [services](http://wiki.ros.org/rosservice) 
and ROS [topics](http://wiki.ros.org/rostopic)
#!bash
rosservice list
```
This should output these services:
```
#!bash

/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/switch_controller
/controller_manager/unload_controller
```
If you call for the controller types:
```
#!bash

rosservice call /controller_manager/list_controller_types
```
This should show our custom controller plugin:
```
#!bash
types: ['hw_controller/singleJointController']
base_classes: ['controller_interface::ControllerBase']
```

### Status of the controller ###
The status of the controller can be queried via:
```
#!bash
rosservice call /controller_manager/list_controllers
```
This should show the running controllers, together with the resources they have been assigned to
```
#!bash
controller: 
  - 
    name: test_controller0
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor0']
  - 
    name: test_controller1
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor1']
  - 
    name: test_controller2
    state: running
    type: hw_controller/singleJoint_controller
    hardware_interface: hardware_interface::PositionJointInterface
    resources: ['motor2']

```

# Test with hardware #
please follow instructions in [flexrayusbinterface](https://gitlab.lrz.de/rosifyingmyorobotics/flexrayusbinterface), concerning library installation and udev rule.
Use initialize service to initilaize a motor X (see above), then use the trajectory service
```
#!bash
rosservice call /roboy/trajectory_motorX
```