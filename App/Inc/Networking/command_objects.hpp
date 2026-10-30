#pragma once

#include "borbes_shorthands.h"

namespace Commands {

#pragma pack(push, 1)

enum class StateCommands : uint8_t {
    IDLE                 = 0,
    OPEN_FILLING         = 1,
    OPEN_SEPARATION_FILL = 2,
    EXPECT_LIFTOFF       = 3,
    OPEN_FLOTATION_FILL  = 4,
    OPEN_SEPARATION      = 5,
};

enum class OtherCommand : uint8_t {
    TOGGLE_CAMERA = 0,
    FORMAT_SD     = 1,
};

/**
 * @commands
 * #id=0x10
 */
struct FSMStateCommand {
    StateCommands command;
};

/**
 * @commands
 * #id=0x11
 */
struct GroundStationHeartBeat {
    uint8_t ignore;
};

/**
 * @commands
 * #id=0x12
 */
struct OtherCommandsGUI {
    OtherCommand cmd;
};

#pragma pack(pop)

}; // namespace Commands
