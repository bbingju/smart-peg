#ifndef EVENT_SOURCE_H_
#define EVENT_SOURCE_H_

#include "esp_event.h"
#include "esp_event_loop.h"

#ifdef __cplusplus
extern "C" {
#endif

// Declarations for the event source
struct peg_event_arg {
    int argnum;
    uint32_t args [8];
};

ESP_EVENT_DECLARE_BASE(PEG_EVENTS);         // declaration of the task events family

enum {
    PEG_EVENT_LED_SET_DIRECT_DRAW,  //
    PEG_EVENT_LED_DRAW,
    PEG_EVENT_LED_CLEAR,        //
    PEG_EVENT_LED_SET_PIXEL,    //
    PEG_EVENT_LED_FILL_RECT,

    PEG_EVENT_MAG_STATUS,
};

#ifdef __cplusplus
}
#endif

#endif // #ifndef EVENT_SOURCE_H_
