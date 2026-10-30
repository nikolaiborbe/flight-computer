#pragma once

#include "pcan_blinky.h"
#include "pcan_core.hpp"

class PCanBlinky : public PCanPlugin {
    pcan_blinky_instance_t blinky;

  public:
    PCanBlinky(pcan_blinky_led_t led) { pcan_blinky_create(&this->blinky, led); }

    virtual int register_plugin(PCan &pcan) override {
        return pcan_blinky_register_plugin(pcan.get_instance(), &this->blinky);
    }
};
