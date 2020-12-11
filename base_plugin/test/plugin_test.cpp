#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include "base_plugin/base_plugin.hpp"


/**
 * Test for loading unknown plugin.
 */
TEST(pluginTest, unknownPlugin)
{
    // Instantiate plugin loader
    pluginlib::ClassLoader<base_plugin::main_plugin_class> plugin_loader("base_plugin", "base_plugin::main_plugin_class");

    // Expect error thrown when unknown plugin is loaded
    ASSERT_THROW(
        // Load unknown plugin
        auto unknownplugin = plugin_loader.createInstance("unknown_namespace::unknown_plugin"),
        pluginlib::LibraryLoadException
    );
}


/**
 * Test for unloading plugin.
 */
TEST(pluginTest, unloadPlugin)
{
    // Instantiate plugin loader
    pluginlib::ClassLoader<base_plugin::main_plugin_class> plugin_loader("base_plugin", "base_plugin::main_plugin_class");

    // Plugin name
    std::string plugin_name {"feature_one::feature_one_plugin"};

    // Expect no error thrown when unloading plugin
    ASSERT_NO_THROW(
        // Load plugin
        auto feature_one = plugin_loader.createInstance(plugin_name);
        // These action ought to be performed together
        {
            // Delete pointer to plugin before unloading
            feature_one.reset();
            // Unload plugin
            plugin_loader.unloadLibraryForClass(plugin_name);
        }
    );

}


/**
 * Run all tests
 */
int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}