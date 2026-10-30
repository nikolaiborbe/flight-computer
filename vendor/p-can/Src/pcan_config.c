#include "pcan_config.h"

#define PCAN_CONFIG_MAGIC_FIRST  (0x5043414E) // "PCAN"
#define PCAN_CONFIG_MAGIC_SECOND (0x434F4E46) // "CONF"

typedef struct {
    uint32_t magic_first;
    uint32_t magic_second;
    uint32_t suffix_offset;
    pcan_config_data_t data;
    uint32_t magic_first_inverse;
    uint32_t magic_second_inverse;
} pcan_config_storage_t;

static const pcan_config_data_t default_config_data = {
    .id = {7, 7},
};

static const volatile pcan_config_storage_t config_storage = {
    .magic_first          = PCAN_CONFIG_MAGIC_FIRST,
    .magic_second         = PCAN_CONFIG_MAGIC_SECOND,
    .suffix_offset        = offsetof(pcan_config_storage_t, magic_first_inverse),
    .data                 = default_config_data,
    .magic_first_inverse  = ~PCAN_CONFIG_MAGIC_FIRST,
    .magic_second_inverse = ~PCAN_CONFIG_MAGIC_SECOND,
};

int pcan_config_read(pcan_config_data_t *const out_config) {
    if (~config_storage.magic_first != ~PCAN_CONFIG_MAGIC_FIRST ||
        ~config_storage.magic_second != ~PCAN_CONFIG_MAGIC_SECOND ||
        config_storage.suffix_offset != sizeof(pcan_config_data_t) ||
        config_storage.magic_first_inverse != ~PCAN_CONFIG_MAGIC_FIRST ||
        config_storage.magic_second_inverse != ~PCAN_CONFIG_MAGIC_SECOND) {
        *out_config = default_config_data;
        return 0;
    }

    *out_config = config_storage.data;

    return 0;
}