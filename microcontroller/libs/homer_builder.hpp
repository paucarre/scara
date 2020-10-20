#pragma once

#include "linear_homer.hpp"
#include "rotary_homer.hpp"
#include "actuator_type.hpp"

static RotaryHomer rotary_homer;
static LinearHomer linear_homer;
class HomerBuilder {

    public:
        static Homer* build_homer(ActuatorType actuator_type) {
            if(actuator_type == ActuatorType::ROTARY) {
                return &rotary_homer;
            } else {
                return &linear_homer;
            }
        }
};