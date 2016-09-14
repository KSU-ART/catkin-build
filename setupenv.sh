#!/bin/bash	
#Install additional packages
apt update && \
apt -y install git vim cmake catkin python3-dev python3-numpy dos2unix curl && \

#Install ROS Kinetic
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116 && \
apt update && \
apt -y install ros-kinetic-ros-core && \
rosdep init && \
rosdep update && \
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \

#install ros packages
apt -y install ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-mavlink ros-kinetic-mavros ros-kinetic-mavros-msgs \
    ros-kinetic-cmake-modules ros-kinetic-control-toolbox && \

#Setup Catkin Workspace
mkdir -p ~/catkin_ws/src && \
cd ~/catkin_ws/src && \
catkin_init_workspace && \
catkin_make -j $(nproc) -C ~/catkin_ws/ && \
#exec bash && \

#Install OpenCV
cd && \
git clone https://github.com/Itseez/opencv.git && \
#git clone https://github.com/Itseez/opencv_contrib.git && \
#cd opencv_contrib && \
#git checkout 3.1.0 && \
cd ~/opencv && \
git checkout 3.1.0 && \
mkdir build && \
cd build && \
#	-D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules \
cmake -D CMAKE_BUILD_TYPE=MinSizeRel  \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D BUILD_TESTS=OFF \
	-D BUILD_PYTHON_SUPPORT=ON \
	.. && \
make -j $(nproc) && \
make -j $(nproc) install && \
ldconfig && \

#Install PID
cd && \
git clone https://github.com/silentreverb/pid-controller.git && \
mkdir pid-controller/build && \
cd pid-controller/build && \
cmake .. && \
make -j $(nproc) &&
make -j $(nproc) install && \

#Install Atlante
cd && \
git clone https://github.com/uavster/atlante.git && \
cd atlante && \
make -j $(nproc) install && \

#Vim Plugin Manager
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim && \