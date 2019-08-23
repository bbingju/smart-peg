#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

#define DEFAULT_PIXEL_WIDTH   10
#define DEFAULT_PIXEL_HEIGHT  10

typedef enum leds_event_type {
    LEDS_EVENT_TYPE_NONE,
    LEDS_EVENT_SET_COLOR,
    LEDS_EVENT_REQ_STATUS,
} leds_event_type_t;

typedef uint32_t color_t;

struct pixel {
    int id;
    int x, y;
    color_t color;
};

typedef struct leds * leds_handle_t;

#ifdef __cplusplus
#endif

extern leds_handle_t leds_create(int width, int height);
extern void leds_destroy(leds_handle_t handle);

extern void leds_clear(leds_handle_t handle);
extern void leds_set_color(leds_handle_t handle, int x, int y, color_t color);
extern void leds_fill_rect(leds_handle_t handle, int x1, int y1, int x2, int y2, color_t color);

extern void leds_draw(leds_handle_t handle);

#ifdef __cplusplus
#endif

#endif /* LEDS_H */
