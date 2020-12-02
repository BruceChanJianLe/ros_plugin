# ROS Plugin

This repository demonstrates the usage of plugins in ROS packages.

Note that the ClassLoader must not go out scope while you are using the plugin. So, if you are loading a plugin object inside a class, make sure that the class loader is a member variable of that class. 

## Content
- [Plugin Description File Reference](#Plugin-Description-File-Reference)

## Plugin Description File Reference

Tag | Explanation
---|---
`<class_libraries>` | The **class_libraries** tag allows listing multiple libraries which, in turn, contain plugins within one plugin description file.
`<library>` | The **library** tag defines the library in which plugin classes live. A library may contain multiple plugins of varying class types.
`<class>` | The **class** tag describes a class provided by a library. 

For `<class>` Tag:
- `name` : The lookup name of the class. Used by the [pluginlib](http://roswiki.autolabor.com.cn/pluginlib.html) tools as an identifier for the plugin. This field is optional in pluginlib 1.9 and higher (ROS Groovy or higher) 
- `type` : The fully qualified class type. 
- `base_class_type` : The fully qualified type of the base class
- `description` : A description of the class and what it does. 

Example: A simple plugin description file for a library with one class to export 
```xml
<library path="lib/libsingle_obstacle_layer">
  <class type="ros_costmap_plugin_namespace::single_robot_front_obstacle_layer" base_class_type="costmap_2d::Layer">
    <description>A layer that marks a one meter in front of the robot as obstacle</description>
  </class>
</library>
```

Example: A plugin description for multiple libraries with plugins 
```xml
<class_libraries>

  <library path="lib/libsingle_obstacle_layer">
    <class type="ros_costmap_plugin_namespace::single_robot_front_obstacle_layer" base_class_type="costmap_2d::Layer">
      <description>A layer that marks a one meter in front of the robot as obstacle</description>
    </class>
  </library>

  <library path="lib/liball_obstacle_layer">
    <class type="ros_costmap_plugin_namespace::all_robot_front_obstacle_layer" base_class_type="costmap_2d::Layer">
      <description>A layer that marks all points that were ever one meter in front of the robot as obstacle</description>
    </class>
  </library>

  <library path="lib/libinteractive_obstacle_layer">
    <class type="ros_costmap_plugin_namespace::interactive_layer" base_class_type="costmap_2d::Layer">
      <description>A layer that mark interactive marker point as an obstacle</description>
    </class>
  </library>

</class_libraries>
```

Example: A plugin description file for a library with multiple plugins
```xml
<library path="lib/libplugin">
  <class name="FirstPlugin" type="my_namespace::FirstPlugin" base_class_type="interface_namespace::PluginInterface">
    <description>
      A description of FirstPlugin
    </description>
  </class>
  <class name="SecondPlugin" type="my_namespace::SecondPlugin" base_class_type="interface_namespace::PluginInterface">
    <description>
      A description of SecondPlugin
    </description>
  </class>
</library>
```

## Reference

- Introduction of ROS plugin with Packt [link](https://hub.packtpub.com/working-pluginlib-nodelets-and-gazebo-plugins/)
- Meaning of plugin.xml tags [link](http://roswiki.autolabor.com.cn/pluginlib(2f)PluginDescriptionFile.html)
- Implementation and Explanation of pluginlib [link](https://www.xuningyang.com/2020-05-12-ros-pluginlib/)