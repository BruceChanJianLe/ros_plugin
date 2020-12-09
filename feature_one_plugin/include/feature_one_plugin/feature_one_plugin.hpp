#ifndef feature_one_plugin_H_
#define feature_one_plugin_H_

#include <base_plugin/base_plugin.hpp>

namespace feature_one
{
    // Note that the inherited base class must declare public
    class feature_one_plugin: public base_plugin::main_plugin_class
    {
        public:
            feature_one_plugin();
            ~feature_one_plugin();
            virtual void initialize(double length) override;
            virtual double area() override;
        private:
            double length_;
    };
} // namespace feature_one


#endif
