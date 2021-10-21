#include "base_plugin/plugin_loader.hpp"


namespace plugin_loader
{
    user_plugin::user_plugin()
    :   plugin_loader_ptr (std::make_unique<pluginlib::ClassLoader<base_plugin::main_plugin_class>> ("base_plugin", "base_plugin::main_plugin_class")),
        nh_private_ ("~")
    {
        if(nh_private_.hasParam("plugins"))
        {
            ROS_INFO_STREAM("Has Param plugins.");
            // Yaml param holder variable
            XmlRpc::XmlRpcValue plugin_list;

            // Load param from yaml to local variable
            if(nh_private_.getParam("plugins", plugin_list))
            {
                ROS_INFO_STREAM("Obtaining params!");
                try
                {
                    if(plugin_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        for(auto itr = plugin_list.begin()->first.begin(); itr != plugin_list.begin()->first.end(); ++itr)
                        {
                            // std::map<std::string, std::string> map = (std::map<std::string, std::string>)itr;
                            ROS_INFO_STREAM("HERE1!");
                            ROS_INFO_STREAM(*itr);
                            // for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator itr2 = itr->second.begin(); itr2 != itr->second.end(); ++itr2)
                            // {
                            //     ROS_INFO_STREAM((std::string)(itr2->second));
                            // }
                        //     ROS_INFO_STREAM("HERE2!");
                        }
                        // for(int i = 0; i < plugin_list.size(); i++)
                        // {
                        //     ;
                        //     // Insert plugin to map
                        //     plugins_.insert(std::make_pair(plugin_list[i]["name"], plugin_loader_ptr->createInstance(plugin_list[i]["type"])));
                        //     // Emplace names to list
                        //     plugin_names_.push_back(plugin_list[i]["name"]);
                        //     // Inform user of loaded plugin
                        //     ROS_INFO_STREAM(
                        //         "Loaded " << plugin_list[i]["name"] << " plugin!"
                        //     );
                        // }
                    }
                    else
                    {
                        ROS_INFO_STREAM("Did not do anything...");
                    }
                    
                }
                catch(const XmlRpc::XmlRpcException & e)
                {
                    ROS_ERROR_STREAM(ros::this_node::getName() << " caught an error: " << e.getMessage());
                }
            }
            else
            {
                ROS_INFO_STREAM("Failed to getParam!");
            }
        }
        else
        {
            ROS_INFO_STREAM("does not have param plugins!");
        }
        

        // Initialize all loaded plugins
        // for(auto plugin : plugins_)
        // {
        //     plugin.second->initialize(10.0);
        // }
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
