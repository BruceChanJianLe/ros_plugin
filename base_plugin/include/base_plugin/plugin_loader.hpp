#include <ros/ros.h>
#include "base_plugin/base_plugin.hpp"
#include <pluginlib/class_loader.hpp>

#include <memory>
#include <vector>
#include <string>


namespace plugin_loader
{
    class user_plugin
    {
        public:
            user_plugin();
            ~user_plugin();

            void start();
        private:
            ros::NodeHandle nh_private_;
            std::unique_ptr<pluginlib::ClassLoader<base_plugin::main_plugin_class>> plugin_loader_ptr;
            std::vector<boost::shared_ptr<base_plugin::main_plugin_class>> plugins_;

    };
} // namespace plugin_loader
