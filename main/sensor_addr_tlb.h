#ifndef SENSOR_ADDR_TLB_H
#define SENSOR_ADDR_TLB_H

#include <stdint.h>

struct sensor_addr_tlb {
    int id;
    uint8_t address;
};

#define SENSOR_NUMBER 1

struct sensor_addr_tlb sensor_addr_tlb[SENSOR_NUMBER] = {
    { 0, 0x20 },
};

#endif /* SENSOR_ADDR_TLB_H */
