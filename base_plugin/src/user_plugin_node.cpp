#include "base_plugin/user_plugin.hpp"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "user_plugin_node");

    user_namespace::plugin_user node;

    node.run();

    return 0;
}