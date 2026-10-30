#include "Drivers/Sensors/Mmc.hpp"
#include "spi.h"

namespace Driver::Sensors {
namespace MMC {
namespace {
/* constexpr Mmc5983maVector MAG_OFFSET {
    -1.069178913f,
    0.2721459083f,
    0.6913307236f,
};

constexpr float MAG_TRANSFORM[3][3] {
    {1.284890777f, 0.5507179696f, 0.5774966487f},
    {0.5507179696f, 1.397791022f, -0.2691479408f},
    {0.5774966487f, -0.2691479408f, 1.12647892f},
}; */
constexpr Mmc5983maVector MAG_OFFSET {
    -0.2430090783f,
    -0.7522196828f,
    0.05644039269f,
};

constexpr float MAG_TRANSFORM[3][3] {
    { 1.099149135f, 0.02574751904f, -0.04347021295f },
    { 0.02574751904f, 1.004147895f, -0.01271196731f },
    { -0.04347021295f, -0.01271196731f, 1.085356211f },
};
} // namespace

MMC5983MA init()
{
    const Mmc5983maSpiConfig mmc_spi_config {
        .cs_port = SPI2_CS_MMC_GPIO_Port, .cs_pin = SPI2_CS_MMC_Pin, .hspi = &hspi2
    };

    MMC5983MA mmc(mmc_spi_config);
    mmc.init();
    return mmc;
}

void linear_transform_mag(Mmc5983maData& data)
{
    if (!data.valid) {
        return;
    }

    const float x = data.magnetic_gauss.x - MAG_OFFSET.x;
    const float y = data.magnetic_gauss.y - MAG_OFFSET.y;
    const float z = data.magnetic_gauss.z - MAG_OFFSET.z;

    data.magnetic_gauss.x = MAG_TRANSFORM[0][0] * x + MAG_TRANSFORM[0][1] * y + MAG_TRANSFORM[0][2] * z;
    data.magnetic_gauss.y = MAG_TRANSFORM[1][0] * x + MAG_TRANSFORM[1][1] * y + MAG_TRANSFORM[1][2] * z;
    data.magnetic_gauss.z = MAG_TRANSFORM[2][0] * x + MAG_TRANSFORM[2][1] * y + MAG_TRANSFORM[2][2] * z;
}
} // namespace MMC
} // namespace Driver::Sensors
