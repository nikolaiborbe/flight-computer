#pragma once

namespace proto::can::thermocouple {
    constexpr size_t thermocouple_batch_size = 8;
    
    enum class Id : uint8_t {
        OxTankBottom = 0,
        OxTankTop = 1,
        FuelTank = 2,
        OxInjector = 3,
        // Chamber1 = 4,
        FuelInjector = 5,
        // Chamber2 = 6,
        // ExtraNodeOne = 7,
        Chamber0 = 8,
        Chamber1 = 9,
        Chamber2 = 10,
        Chamber3 = 11,

        Count,
    };

    #pragma pack(push, 1)
    struct ThermocoupleSample {
        Id id;
        uint64_t timestamp_us;
        float interval_us;
        float temperature_c[thermocouple_batch_size];
    };
    #pragma pack(pop)
}
