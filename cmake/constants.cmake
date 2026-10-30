# ------------------------------------------------------------------------------
# |> CORE PATHS
# ------------------------------------------------------------------------------
# --- [ Application & Vendor ] ---
set(PATH_APP "${CMAKE_SOURCE_DIR}/App")
set(PATH_VENDOR "${CMAKE_SOURCE_DIR}/vendor")
# --- [ Drivers & RTOS ] ---
set(PATH_STM32_DRIVER "${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver")
set(PATH_FILEX "${CMAKE_SOURCE_DIR}/FileX")
set(PATH_NETXDUO "${CMAKE_SOURCE_DIR}/NetxDuo")
set(PATH_AZURE_RTOS "${CMAKE_SOURCE_DIR}/AZURE_RTOS")
# --- [ Middlewares ] ---
set(PATH_STM_MIDDLEWARES "${CMAKE_SOURCE_DIR}/Middlewares/ST")
set(PATH_MIDDLEWARES_THREADX "${PATH_STM_MIDDLEWARES}/threadx")
set(PATH_MIDDLEWARES_NETXDUO "${PATH_STM_MIDDLEWARES}/netxduo")
set(PATH_MIDDLEWARES_FILEX "${PATH_STM_MIDDLEWARES}/filex")
# --- [ Build Outputs ] ---
set(PATH_PRECOMPILED_LIBRARY "${CMAKE_SOURCE_DIR}/.lib")

# ------------------------------------------------------------------------------
# |> FILE SEARCH PATTERNS (GLOBS)
# ------------------------------------------------------------------------------
set(SOURCE_DIRS_GLOB_EXPRESSIONS
    "${PATH_APP}/Src/**/*.cpp"
    "${PATH_APP}/Src/**/*.c"
    "${PATH_APP}/Src/*.cpp"
    "${PATH_APP}/Src/*.c"
    "${PATH_VENDOR}/**/*.cpp"
    "${PATH_VENDOR}/**/*.c"
)
set(INCLUDE_DIRS_GLOB_EXPRESSIONS
    "${PATH_APP}/Inc/**/*.hpp"
    "${PATH_APP}/Inc/**/*.h"
    "${PATH_VENDOR}/**/*.hpp"
    "${PATH_VENDOR}/**/*.h"
)

# ------------------------------------------------------------------------------
# |> FILTERING & EXCLUSIONS
# ------------------------------------------------------------------------------
set(EXCLUDED_FOLDERS
    "BatteryMonitor"
    "Flash"
    "Capacitance"
)
set(EXCLUDED_FILES
    "adc_manager"
    "toggle_solenoid.h"
    "toggle_solenoid.cpp"
    "MS5611.hpp"
    "MS5611.cpp"
)
set(REGEX_REMOVE_TEMPLATE_FILE_ENDING ".*_template\\.c$")
set(REGEX_NONE "IMPOSSIBLE_REGEX_MATCH_XYZ")

# ==============================================================================
# |> CUBEMX MIRROR
# ==============================================================================

set(MX_Defines_Syms
    USE_PWR_LDO_SUPPLY
    TX_INCLUDE_USER_DEFINE_FILE
    FX_INCLUDE_USER_DEFINE_FILE
    LX_INCLUDE_USER_DEFINE_FILE
    NX_INCLUDE_USER_DEFINE_FILE
    USE_HAL_DRIVER
    STM32H723xx
    $<$<CONFIG:Debug>:DEBUG>
)

set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/AZURE_RTOS/App
    ${CMAKE_SOURCE_DIR}/FileX/App
    ${CMAKE_SOURCE_DIR}/LevelX/App
    ${CMAKE_SOURCE_DIR}/FileX/Target
    ${CMAKE_SOURCE_DIR}/NetXDuo/App
    ${CMAKE_SOURCE_DIR}/NetXDuo/Target
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/filex/common/inc/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/filex/ports/generic/inc/
    ${CMAKE_SOURCE_DIR}/Drivers/BSP/Components/lan8742/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/netxduo/common/drivers/ethernet/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/netxduo/common/inc/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/levelx/common/inc/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/threadx/common/inc/
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/
)

set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/AZURE_RTOS/App/app_azure_rtos.c
    ${CMAKE_SOURCE_DIR}/NetXDuo/App/app_netxduo.c
    ${CMAKE_SOURCE_DIR}/FileX/App/app_filex.c
    ${CMAKE_SOURCE_DIR}/Core/Src/tx_initialize_low_level.S
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Core/Src/app_threadx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/adc.c
    ${CMAKE_SOURCE_DIR}/Core/Src/eth.c
    ${CMAKE_SOURCE_DIR}/Core/Src/fdcan.c
    ${CMAKE_SOURCE_DIR}/Core/Src/octospi.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sdmmc.c
    ${CMAKE_SOURCE_DIR}/Core/Src/spi.c
    ${CMAKE_SOURCE_DIR}/Core/Src/tim.c
    ${CMAKE_SOURCE_DIR}/Core/Src/usart.c
    ${CMAKE_SOURCE_DIR}/Core/Src/usb_otg.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h7xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h7xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32h7xx_hal_timebase_tim.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/startup_stm32h723xx.s
)
