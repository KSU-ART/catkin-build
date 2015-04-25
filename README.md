## Pushing to this repository

Please follow the guidelines below when committing to this repository.

### 1. Do not commit experimental/untested code

Please only commit stable/tested code to this repository. For
experimental changes or untested bug fixes, please fork this repository
and commit your changes there. Once you have verified that your code
works and does not break anything else, create a pull request.

### 2. Follow the commit message style guide

Please follow this style guide when commiting changes to this
repository. It will make it easier for others to determine exactly what
and why something was changed.

http://chris.beams.io/posts/git-commit/

### 3. Keep this guide up to date

This is the most important rule! Make sure to keep this guide up to date
so that future Control Systems members will be able to continue our work
when we are gone.

## Installing and configuring the code base

Ok guys, we have a working code base. Everyone should be able to set up
a local git repository, pull code, build, and execute everything. Here
is a step by step process to setup the code base on your machine.

### 1. Install ROS

Copy and paste the following series of commands to get a ROS
installation up and running. You will need a machine running the Ubuntu
operating system, or an Ubuntu VM running on Windows.

Add the ROS package repository:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release
-sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up keys:

```
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key
0xB01FA116
```

Update package cache:

```
sudo apt-get update
```

Install ROS:

```
sudo apt-get install ros-indigo-desktop-full
```

Set up `rosdep`:

```
sudo rosdep init
rosdep update
```
Add ROS environment variables to your `.bashrc`:

```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Get `rosinstall`:

```
sudo apt-get install python-rosinstall
```

### 2. Set up your catkin workspace

This is where all the packages and source code in this repository will
go. Follow these steps to set it up.

```
mkdir -p ~/ros/catkin_ws/src
cd ~/ros/catkin_ws/src
catkin_init_workspace
```

Test build the workspace to make sure everything is working properly up
to this point.

```
cd ~/ros/catkin_ws/
catkin_make
```

*OPTIONAL: Add `cm` to your `.bash_aliases` to make your life easier.
This allows you to build your catkin workspace from any directory. Make
sure `source ~/.bash_aliases` is in your `.bash_rc` to use.*

```
echo "aliases cm = 'catkin_make -C ~/ros/catkin_ws/'" >> ~/.bash_aliases
source ~/.bash_aliases
```

### 3. Install & build the necessary dependencies

Almost done! First you need to install and build several packages before
the source code can be succesfully built. If any other dependencies are
required in the future, please add the steps to install and build them
here.

Install dependencies obtainable through `apt-get`:

```
sudo apt-get install build-essential cmake-modules git libusb1.0-0-dev
libeigen3-dev ros-indigo-cv-bridge ros-indigo-image-transport
ros-indigo-tf mavros
```

Download and compile the phidgets driver:

```
cd ~/
wget http://www.phidgets.com/downloads/libraries/libphidget.tar.gz
tar -zxvf libphidget.tar.gz
...To be continued...
```

Download and compile OpenCV:

```
...To be continued...
```


### 4. Clone the repository and build the code

Clone the repository in the catkin workspace:

```
cd ~/ros/catkin_ws/
git clone https://github.com/SPSU-ART/catkin-build.git src
```

Build the code using `catkin_make` or `cm`. If everything compiles,
you're done! Now code something useful for the team, darn it.


## Flight execution instructions for autonomous flight

Make sure roscore is running:

```
roscore &
```

Run the mavros program which is a ros mavlink interface that provides
ROS topics for Mavlink communication:

```
roslaunch mavros apm2.launch
```

If Pixhawk make sure to set the ground control system id to 1. The
pixhawk will ignore rc override messages otherwise

```
rosrun mavros mavparam set SYSID_MYGCS 1
```

If APM 2.x set the stream rate to 10 or 5 this is the only rate we've
found that will allow for sensor feedback and rc sending

```
rosrun mavros mavsys rate --all 10
```

Lastly, after ensuring all neccessary sensors are running (e.g Xtion Pro
Live, UVC_camera, px4flow). Run an autonomous flight program (e.g
mavros_flight_test)

```
rosrun spsuart mavros_flight_test
```

