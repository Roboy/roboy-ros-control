FROM ros:jade
RUN apt-get update
RUN apt-get install build-essential -y
RUN apt-get install -y ros-jade-desktop
RUN apt-get install -y ros-jade-controller-interface ros-jade-controller-manager ros-jade-control-toolbox ros-jade-transmission-interface ros-jade-joint-limits-interface 
RUN apt-get install wget
RUN wget -O /tmp/gazebo6_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo6_install.sh && sh /tmp/gazebo6_install.sh

ENV ROS_ROOT=/opt/ros/jade/share/ros
ENV ROS_PACKAGE_PATH=/root/catkin_ws/src:/opt/ros/jade/share:/opt/ros/jade/stacks
ENV ROS_MASTER_URI=http://localhost:11311
ENV LD_LIBRARY_PATH=/root/catkin_ws/devel/lib:/root/catkin_ws/devel/lib/x86_64-linux-gnu:/opt/ros/jade/lib/x86_64-linux-gnu:/opt/ros/jade/lib
ENV CATKIN_TEST_RESULTS_DIR=/root/catkin_ws/build/test_results
ENV CPATH=/root/catkin_ws/devel/include:/opt/ros/jade/include
ENV ROS_TEST_RESULTS_DIR=/root/catkin_ws/build/test_results
ENV PATH=/root/catkin_ws/devel/bin:/opt/ros/jade/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV ROSLISP_PACKAGE_DIRECTORIES=/root/catkin_ws/devel/share/common-lisp
ENV ROS_DISTRO=jade
ENV PYTHONPATH=/root/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/jade/lib/python2.7/dist-packages
ENV PKG_CONFIG_PATH=/root/catkin_ws/devel/lib/pkgconfig:/root/catkin_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/jade/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/jade/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=/root/catkin_ws/devel:/opt/ros/jade
ENV ROS_ETC_DIR=/opt/ros/jade/etc/ros

RUN apt-get install -y ros-jade-driver-base ros-jade-polled-camera ros-jade-camera-info-manager ros-jade-gazebo-ros-pkgs
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs && cd gazebo_ros_pkgs && git checkout jade-devel
RUN git clone https://github.com/andreasBihlmaier/gazebo2rviz.git 
RUN git clone https://github.com/Roboy/pysdf
WORKDIR /root/catkin_ws
RUN catkin_make_isolated --install --install-space /opt/ros/jade/ -DCMAKE_BUILD_TYPE=Release
RUN rm -r /root/catkin_ws/
RUN git clone https://github.com/Roboy/ros_control /root/catkin_ws && git checkout gazebo_integration && git submodule init && git submodule update
# install ftd2xx driver
RUN cd src/flexrayusbinterface/lib && dpkg -i libftd2xx_1.1.12_amd64.deb
#RUN cd src/myo_master/patches && diff -u /usr/include/FreeImage.h FreeImage.h > FreeImage.diff && patch /usr/include/FreeImage.h < FreeImage.diff
RUN catkin_make --pkg common_utilities
RUN catkin_make
RUN ln -s /root/catkin_ws ~/catkin_ws
