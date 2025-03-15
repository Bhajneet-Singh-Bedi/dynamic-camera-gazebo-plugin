## What this repo contains ??
A gazebo ignition and ros2 humble plugin ```ros2_topic_publisher.cc```. This plugin will dynamically change the position of gazebo camera. 

It subscribes to /pose topic of ros2 script of type ```geometry_msgs/msg/Pose```. 

Steps to use this plugin:- 
 - Build:- 
```bash
  mkdir build
  cd build
  cmake ..
  make 
```
 - A ```libros2_topic_subscriber.so``` binary file will be created.
 - Add this to zshrc or bashrc 👉```export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=full/path/to/libros2_topic_subscriber.so:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH```
 - With this command ignition gazebo will be able to identify new plugin added to the system.
 - To use this plugin in sdf file, do this 👇
 ```
<model name="my_model">
   ...
   ...
   <plugin name="ros2_topic_subscriber" filename="ros2_topic_subscriber">
      <topic>/pose</topic>
   </plugin>
</model>
```

Peace Out ✌️
