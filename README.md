# ROS Plugin

This repository demonstrates the usage of plugins in ROS packages.

Note that the ClassLoader must not go out scope while you are using the plugin. So, if you are loading a plugin object inside a class, make sure that the class loader is a member variable of that class. 

## Content
- [Plugin Description File Reference](#Plugin-Description-File-Reference)
- [Verify Plugin Declaration and Path](#Verify-Plugin-Declaration-and-Path)
- [ERROR](#ERROR)

## Creating a Base Plugin Package

**Step 1:**  
Create a package for base plugin class with only a single header file. Note that this package will also be using all the other plugins that you have defined. For now just create a base plugin class.

```bash
catkin_create_pkg base_plugin roscpp rospy pluginlib
```

**Step 2:**  
Fill up the header file with methods or members that you wish to have in all your plugins.
```cpp
#ifndef base_plugin_H_
#define base_plugin_H_

#include <ros/ros.h>
#include <memory>

namespace base_plugin
{
    class main_plugin_class
    {
        public:
            virtual void initialize(double length) = 0;
            virtual double area() = 0;
            virtual ~main_plugin_class() {};

        protected:
            main_plugin_class() {};
    };
} // namespace test_base_plugin

#endif
``` 

## Creating Plugins

**Step 1:**  
Create a pacakge with base plugin class as dependency.  

```bash
catkin_create_pkg feature_one_plugin roscpp rospy pluginlib base_plugin
```

**Step 2:**  
`Include` base plugin header file in feature_one_plugin and use the `override` keyword.  
```cpp
#ifndef feature_one_plugin_H_
#define feature_one_plugin_H_

#include <base_plugin/base_plugin.hpp>

namespace feature_one
{
    // Note that the inherited base class must declare public
    class feature_one_plugin: public base_plugin::main_plugin_class
    {
        public:
            feature_one_plugin();
            ~feature_one_plugin();
            void initialize(double length) override;
            double area() override;
        private:
            double length_;
    };
} // namespace feature_one
#endif
```

**Step 3:**  
Fill up the source file for the feature one plugin.  

**Step 4:**  
Add plugin.xml file with details.  
```xml
<library path="lib/libfeature_one">

  <class type="feature_one::feature_one_plugin" base_class_type="base_plugin::main_plugin_class">
    <description>This is feature one.</description>
  </class>

</library
```

**Step 5:**  
Add export tag in package.xml file.  
```xml
<export>
  <base_plugin plugin="${prefix}/plugin.xml"/>
</export>
```

**Step 6:**  
Edit CMakeLists.txt by adding the library.  
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(feature_one_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  base_plugin
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES feature_one_plugin
#  CATKIN_DEPENDS pluginlib roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(feature_one src/feature_one_plugin.cpp)

add_dependencies(feature_one ${catkin_EXPORTED_TARGETS})
target_link_libraries(feature_one ${catkin_LIBRARIES} )
install(TARGETS feature_one
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
```

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

## Verify Plugin Declaration and Path

To verify that things are working, first build the workspace and source the resulting setup file, then try running the following rospack command:  
```bash
rospack plugins --attrib=plugin base_plugin
```

## ERROR

`/home/chanjl/plugin_ws/devel/lib/base_plugin/user_plugin: symbol lookup error: /home/chanjl/plugin_ws/devel/lib//libfeature_one.so: undefined symbol: _ZN11base_plugin17main_plugin_classC2Ev`

In this error, it seems like there is a symbol lookup error. And if you run the command `c++filt _ZN11base_plugin17main_plugin_classC2Ev` you will know that which member of the class is missing, in our example here is that we have implemented a `base_plugin.hpp` and `base_plugin.cpp` which is undesirable. Therefore, please only define the `base_plugin.hpp` in the example shown.

For more information please look at this [video](https://www.youtube.com/watch?v=4xZKflNlJho) and [link](https://www.xuningyang.com/2020-05-12-ros-pluginlib/).

## Reference

- Introduction of ROS plugin with Packt [link](https://hub.packtpub.com/working-pluginlib-nodelets-and-gazebo-plugins/)
- Meaning of plugin.xml tags [link](http://roswiki.autolabor.com.cn/pluginlib(2f)PluginDescriptionFile.html)
- Implementation and Explanation of pluginlib [link](https://www.xuningyang.com/2020-05-12-ros-pluginlib/)
- Symbol Lookup Error [link](https://www.youtube.com/watch?v=4xZKflNlJho)
