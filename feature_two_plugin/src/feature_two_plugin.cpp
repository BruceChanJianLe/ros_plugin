#include "feature_two_plugin/feature_two_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(feature_two::feature_two_plugin, base_plugin::main_plugin_class);

namespace feature_two
{
    feature_two_plugin::feature_two_plugin()
    {
        ;
    }


    feature_two_plugin::~feature_two_plugin()
    {
        ;
    }


    void feature_two_plugin::initialize(double length)
    {
        length_ = length;
    }


    double feature_two_plugin::area()
    {
        return 2.0 * length_;
    }
} // namespace feature_two
