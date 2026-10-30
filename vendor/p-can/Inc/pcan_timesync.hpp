#include "pcan_core.hpp"
#include "pcan_timesync.h"

class PCanTimesync : public PCanPlugin {
    pcan_timesync_instance_t timesync;

  public:
    PCanTimesync(TIM_HandleTypeDef *htim, pcan_timesync_role_t role) {
        pcan_timesync_create(&this->timesync, htim, role);
    }

    int set_external_clock(uint64_t (*get_time_us)(void *),
                           uint64_t (*set_correction_us)(void *, int64_t), void *user_data) {
        return pcan_timesync_set_external_clock(&this->timesync, get_time_us, set_correction_us,
                                                user_data);
    }

    pcan_timesync_time_t now_us() { return pcan_timesync_now_us(&this->timesync); }

    pcan_timesync_time_t now_ms() { return pcan_timesync_now_ms(&this->timesync); }

    void on_period_elapsed(TIM_HandleTypeDef *htim) {
        pcan_timesync_on_period_elapsed(&this->timesync, htim);
    }

    virtual int register_plugin(PCan &pcan) override {
        return pcan_timesync_register_plugin(pcan.get_instance(), &this->timesync);
    }
};
