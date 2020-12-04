#include "base_plugin/plugin_loader.hpp"


namespace plugin_loader
{
    user_plugin::user_plugin()
    :   plugin_loader_ptr (std::make_unique<pluginlib::ClassLoader<base_plugin::main_plugin_class>> ("base_plugin", "base_plugin::main_plugin_class")),
        nh_private_ ("~")
    {
        if(nh_private_.hasParam("plugins"))
        {
            XmlRpc::XmlRpcValue plugin_list;
            nh_private_.getParam("plugins", plugin_list);
            ROS_INFO_STREAM("size of plugin_list: " << plugin_list.size());
            for(int i = 0; i < plugin_list.size(); i++)
            {
                ROS_INFO_STREAM(
                    "Loaded " << plugin_list[i]["name"] << " plugin!"
                );
                plugins_.emplace_back(plugin_loader_ptr->createInstance(plugin_list[i]["type"]));
            }
        }

        // Initialize all loaded plugins
        for(auto plugin : plugins_)
        {
            plugin->initialize(10.0);
        }
    }


    user_plugin::~user_plugin()
    {
        ;
    }


    void user_plugin::start()
    {
        // Run all loaded plugins
        for(auto plugin : plugins_)
        {
            ROS_INFO_STREAM("Area: " << plugin->area());
        }
    }
} // namespace plugin_loader
