#include "base_plugin/plugin_loader.hpp"


namespace plugin_loader
{
    user_plugin::user_plugin()
    :   plugin_loader_ptr (std::make_unique<pluginlib::ClassLoader<base_plugin::main_plugin_class>> ("base_plugin", "base_plugin::main_plugin_class")),
        nh_private_ ("~")
    {
        if(nh_private_.hasParam("plugins"))
        {
            // Yaml param holder variable
            XmlRpc::XmlRpcValue plugin_list;

            // Load param from yaml to local variable
            nh_private_.getParam("plugins", plugin_list);
            for(int i = 0; i < plugin_list.size(); i++)
            {
                // Insert plugin to map
                plugins_.insert(std::make_pair(plugin_list[i]["name"], plugin_loader_ptr->createInstance(plugin_list[i]["type"])));
                // Emplace names to list
                plugin_names_.push_back(plugin_list[i]["name"]);
                // Inform user of loaded plugin
                ROS_INFO_STREAM(
                    "Loaded " << plugin_list[i]["name"] << " plugin!"
                );
            }
        }

        // Initialize all loaded plugins
        for(auto plugin : plugins_)
        {
            plugin.second->initialize(10.0);
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
            ROS_INFO_STREAM("Area: " << plugin.second->area());
        }
    }
} // namespace plugin_loader
