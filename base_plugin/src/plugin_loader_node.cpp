#include "base_plugin/plugin_loader.hpp"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "plugin_loader_node");

    plugin_loader::user_plugin node;

    node.start();

    ros::waitForShutdown();
}