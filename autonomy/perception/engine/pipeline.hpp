#ifndef AUTONOMY_PERCEPTION_ENGINE_PIPELINE_HPP_
#define AUTONOMY_PERCEPTION_ENGINE_PIPELINE_HPP_

#include <string>

#include "autonomy/perception/engine/observation.hpp"

namespace autonomy::perception::base {

// A provider fills observations for one source RGB frame. The component
// assigns message headers and publishes configured output topics.
class Pipeline {
public:
    virtual ~Pipeline() = default;
    virtual bool Run(const automsgs::msgs::sensor_msgs::Image& rgb,
                     Outputs* outputs, std::string* error) = 0;
};

}  // namespace autonomy::perception::base

#endif  // AUTONOMY_PERCEPTION_ENGINE_PIPELINE_HPP_
