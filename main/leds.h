#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

#define DEFAULT_PIXEL_WIDTH   10
#define DEFAULT_PIXEL_HEIGHT  10

typedef uint32_t color_t;

struct pixel {
    int id;
    int x, y;
    color_t color;
};

typedef struct leds * leds_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

  leds_handle_t leds_create(int width, int height);
  void leds_destroy(leds_handle_t handle);

  void leds_clear(leds_handle_t handle);
  void leds_set_color(leds_handle_t handle, int x, int y, color_t color);
  void leds_fill_rect(leds_handle_t handle, int x1, int y1, int x2, int y2, color_t color);

  void leds_draw(leds_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* LEDS_H */
