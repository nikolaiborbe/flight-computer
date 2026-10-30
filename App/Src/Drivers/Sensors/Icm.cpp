#include "Drivers/Sensors/Icm.hpp"

namespace Driver::Sensors {
namespace ICM {
ICM40609D init()
{
    const IcmSpiConfig icm_spi_config {
        .cs_port = SPI2_CS_ICM_GPIO_Port, .cs_pin = SPI2_CS_ICM_Pin, .hspi = &hspi2
    };

    ICM40609D icm(icm_spi_config);
    icm.init(ACC_OFFSET_X_G, ACC_OFFSET_Y_G, ACC_OFFSET_Z_G);

    return icm;
}
} // namespace ICM
} // namespace Driver::Sensors
