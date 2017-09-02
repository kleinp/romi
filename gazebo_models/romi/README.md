## Building the plugin

Run the following.

```
cd plugin/build
cmake ..
make
```

This should create a shared object in `plugin/build`. That directory is automatically included as _GAZEBO\_PLUGIN\_PATH_ when sourcing the ROS _setup.bash_

