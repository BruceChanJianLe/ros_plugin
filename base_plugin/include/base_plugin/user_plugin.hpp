#ifndef user_plugin_H_
#define user_plugin_H_

#include <ros/ros.h>
#include <base_plugin/base_plugin.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace user_namespace
{
    class plugin_user
    {
        public:
            plugin_user();
            ~plugin_user();
            void run();
    };
} // namespace user_namespace


#endif