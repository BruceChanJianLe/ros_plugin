#ifndef feature_one_plugin_H_
#define feature_one_plugin_H_

#include <base_plugin/base_plugin.hpp>

namespace feature_one
{
    class feature_one_plugin: public base_plugin::main_plugin_class
    {
        public:
            feature_one_plugin();
            ~feature_one_plugin();
            void initialize(double length) override;
            double area() override;
        private:
            double length_;
    };
} // namespace feature_one


#endif