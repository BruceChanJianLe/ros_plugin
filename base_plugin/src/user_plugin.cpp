#include "base_plugin/user_plugin.hpp"


namespace user_namespace
{
    plugin_user::plugin_user()
    {
        ;
    }


    plugin_user::~plugin_user()
    {
        ;
    }


    void plugin_user::run()
    {
        // Here, we have to maintain a plugin loader for the entire duration of the program
        std::unique_ptr<pluginlib::ClassLoader<base_plugin::main_plugin_class>> plugin_loader_ptr_;
        // pluginlib::ClassLoader<base_plugin::main_plugin_class> plugin_loader("base_plugin", "base_plugin::main_plugin_class");
        plugin_loader_ptr_ = std::make_unique<pluginlib::ClassLoader<base_plugin::main_plugin_class>> ("base_plugin", "base_plugin::main_plugin_class");

        try
        {
            boost::shared_ptr<base_plugin::main_plugin_class> feature_one = plugin_loader_ptr_->createInstance("feature_one::feature_one_plugin");
            feature_one->initialize(10.0);

            ROS_INFO_STREAM("Feature One Area: " << feature_one->area());
        }
        catch(pluginlib::PluginlibException & e)
        {
            ROS_ERROR_STREAM("The plugin failed to load for some reason. Error: " << e.what());
        }
        
    }
} // namespace user_namespace

