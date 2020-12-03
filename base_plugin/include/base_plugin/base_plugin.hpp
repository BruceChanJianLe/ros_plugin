#ifndef base_plugin_H_
#define base_plugin_H_

#include <ros/ros.h>
#include <memory>

namespace base_plugin
{
    class main_plugin_class
    {
        public:
            virtual void initialize(double length) = 0;
            virtual double area() = 0;
            virtual ~main_plugin_class() {};

        protected:
            main_plugin_class() {};
    };
} // namespace test_base_plugin

#endif