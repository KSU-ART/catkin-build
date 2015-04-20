# Writing good commit messages
Please follow this style guide when commiting changes to this
repository:

http://chris.beams.io/posts/git-commit/


# Installing and configuring the code base

Ok guys, we have a working code base. Everyone should be able to set up a local git repository, pull code, build, and execute everything. Here is a step by step process to setup the code base on your machine.

...To be completed...


# Flight execution instructions for autonomous flight

Make sure roscore is running:

`roscore &`

Run the mavros program which is a ros mavlink interface that provides ROS topics for Mavlink communication:

`roslaunch mavros apm2.launch`

If Pixhawk make sure to set the ground control system id to 1. The pixhawk will ignore rc override messages otherwise

`rosrun mavros mavparam set SYSID_MYGCS 1`

If APM 2.x set the stream rate to 10 or 5 this is the only rate we've found that will allow for sensor feedback and rc sending

`rosrun mavros mavsys rate --all 10`

Lastly, after ensuring all neccessary sensors are running (e.g Xtion Pro Live, UVC_camera, px4flow). Run an autonomous flight program (e.g mavros_flight_test)

`rosrun spsuart mavros_flight_test`
