#ifndef feature_two_plugin_H_
#define feature_two_plugin_H_


#include <base_plugin/base_plugin.hpp>

namespace feature_two
{
    // Note that the inherited base class must declare public
    class feature_two_plugin : public base_plugin::main_plugin_class
    {
        public:
            feature_two_plugin();
            ~feature_two_plugin();
            virtual void initialize(double length) override;
            virtual double area() override;
        private:
            double length_;
    };
} // namespace feature_two


#endif
