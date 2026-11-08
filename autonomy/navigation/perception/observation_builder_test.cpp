/* Copyright 2026 The Navigation Authors (navigation) */
#include <iostream>

#include "navigation/perception/observation_builder.hpp"

int main()
{
    automsgs::msgs::sensor_msgs::Image depth;
    depth.set_width(2);
    depth.set_height(2);
    depth.set_encoding("32FC1");
    const float values[4] = {1.0F, 1.0F, 1.0F, 1.0F};
    depth.set_data(reinterpret_cast<const char*>(values), sizeof(values));
    depth.mutable_header()->mutable_stamp()->set_sec(1);

    navigation::perception::ObservationInputs inputs;
    inputs.depth = &depth;
    inputs.tf_valid = true;
    navigation::observation::Observation observation;
    navigation::perception::ObservationBuilder builder;
    if (!builder.Build(inputs, &observation))
    {
        std::cerr << "build failed\n";
        return 1;
    }
    if (observation.sequence() != 1 ||
        !(observation.validity() & navigation::observation::OBSERVATION_FLAG_DEPTH_VALID) ||
        !(observation.validity() & navigation::observation::OBSERVATION_FLAG_TF_VALID))
    {
        std::cerr << "validity failed\n";
        return 1;
    }
    if (observation.validity() &
        navigation::observation::OBSERVATION_FLAG_CAMERA_INFO_VALID)
    {
        std::cerr << "missing camera marked valid\n";
        return 1;
    }
    if (builder.Build(navigation::perception::ObservationInputs{}, &observation))
    {
        std::cerr << "empty depth accepted\n";
        return 1;
    }
    return 0;
}
