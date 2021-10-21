#include <ros/ros.h>
#include "base_plugin/base_plugin.hpp"
#include <pluginlib/class_loader.hpp>

#include <memory>
#include <vector>
#include <string>
#include <map>

#include <xmlrpcpp/XmlRpc.h>

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
            std::map<std::string, boost::shared_ptr<base_plugin::main_plugin_class>> plugins_;
            std::vector<std::string> plugin_names_;

    };
} // namespace plugin_loader
