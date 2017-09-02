# Gazebo model
Mostly followed the following tutorials to create the model

http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin

## Building the plugin

Run the following.

```
cd plugin/build
cmake ..
make
```

This should create a shared object in `plugin/build`. That directory is automatically included as _GAZEBO\_PLUGIN\_PATH_ when sourcing the ROS _setup.bash_

