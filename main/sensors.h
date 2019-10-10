#ifndef SENSORS_H
#define SENSORS_H

#include "esp_err.h"

struct sensors_data {
    uint16_t data[11];
};

#ifdef __cplusplus
extern "C" {
#endif

    esp_err_t sensors_init();
    struct sensors_data * sensors_read();

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
