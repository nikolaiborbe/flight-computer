#include "Networking/commands/test_state_command_service.hpp"
#include "Context.hpp"
#include <cstring>

namespace {

#pragma pack(push, 1)
struct WireGnss {
    uint8_t data_valid;
    uint8_t numSV;
    double lat, lon, hMSL, gSpeed, headMot;
    uint32_t hAcc, vAcc;
    int32_t velN, velE, velD;
};

struct WireBme280 {
    uint8_t valid;
    float temperature;
    float pressure;
    float humidity;
    uint32_t timestamp;
};

struct WireMs5611 {
    uint8_t valid;
    float temperature;
    float pressure;
    uint32_t timestamp;
};

struct WireIcm {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature_C;
};

struct WireStim {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
};

struct WirePressure {
    float flotation_pressure;
    float separation_pressure;
};

struct WireSensorPayload {
    WireGnss gnss;
    WireBme280 bme;
    WireMs5611 ms;
    WireIcm icm;
    WireStim stim;
    WirePressure pressure;
};
#pragma pack(pop)

static_assert(sizeof(WireSensorPayload) == 152, "wire layout drifted from tests/main.py");

} // namespace

namespace Network::Services {
namespace {
    constexpr uint8_t kCommandID = 0x20;
}
    TestStateCommandService::TestStateCommandService() noexcept
        : CommandService("TestService", kCommandID)
    {
    }

    void TestStateCommandService::handle(std::span<const uint8_t> payload) noexcept
    {

    }
}
