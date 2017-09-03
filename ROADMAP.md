# Roadmap
Description of work to be done to get to have simulated robot (with hooks for actual hardware) map and navigate a room by itself.

## rviz
While not strickly necessary, It would be great to be able to see the map the robot creates in this program.

- pose: pose of robot
- Odometry: odometry poses -- use this instead of pose?
- laser scan: showing distance sensor readings
- path: path robot is planning on taking

rviz has 'save' and 'load' config options, and interactive loading of topics, so might not have to write any code for this part. When I load a laser scan topic right now, it complains about "For frame [romi_frame]: Fixed Frame [map] does not exist" so need to figure out transform frames. Also "No tf data. Actual error: Fixed Frame [map] does not exist"

## gazebo
Needed for the simulation part of this project. It seems like launching gazebo from ROS makes it include some extra paths that load things like laserScan shared object that don't seem to work when loading gazebo separately. It would also be nice if it started the world with the romi robot and any objects/rooms, etc. to navigate in without having to do it manually. 

The package is `gazebo_ros`. Will want to start ROS using one of these methods from [here](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros#OtherROSWaysToStartGazebo)

```
rosrun gazebo_ros gazebo                // server and client
rosrun gazebo_ros gzserver              // server only
rosrun gazebo_ros empty_world.launch    // load things??
```

Though the [next page](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros) of the tutorials shows a structure of files for including a robot and world in the ROS workspace

```
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
```

And then the world and robot are launched with `roslaunch MYROBOT_gazebo MYROBOT.launch`

## Overall robot
There will be a ROS package that is the robot, with as many 'nodes' as needed. The package will have a [launch](http://wiki.ros.org/roslaunch) file (or several for different configurations) that launches the appropriate nodes. The launch file can also call other launch files.

## Nodes

### robot interface (simulator)
Aggregate the topics from Gazebo that the robot publishes and subscribes to and makes them match what the real hardware would produce

- odometry left (what format is this in?)
- odometry right
- velocity left
- velocity right

- line sensors
- analog channels
    - battery voltage
- digital outputs
- digital inputs

### robot interface (hardware)
Communicates with the microcontroller and publishes or subscribes to the same list of topics as the simulator robot interface

### laser scan (simulator)
Arranges the single point TOF range scans correctly and publishes the right topics

### laser scan (hardware)
I2C communication to the TOF sensors to publish the same topics as the simulation

### SLAM
Builds a map and uses it (along with odometry) to position itself in the 'world'.

### romi_brain
High level logic for what the robot does. Hopefully explorer the room to the most possible extent to build the best map.