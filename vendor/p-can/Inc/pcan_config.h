#ifndef __PCAN_CONFIG_H__
#define __PCAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "pcan_core.h"

typedef struct __attribute__((packed)) {
    pcan_node_id_t id;
} pcan_config_data_t;

int pcan_config_read(pcan_config_data_t *const out_config);

#ifdef __cplusplus
}
#endif

#endif // __PCAN_CONFIG_H__
