#include "feature_one_plugin/feature_one_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(feature_one::feature_one_plugin, base_plugin::main_plugin_class)

namespace feature_one
{
    feature_one_plugin::feature_one_plugin()
    {
        ;
    }


    feature_one_plugin::~feature_one_plugin()
    {
        ;
    }


    void feature_one_plugin::initialize(double length)
    {
        length_ = length;
    }


    double feature_one_plugin::area()
    {
        return 0.5 * length_;
    }
} // namespace feature_one

