/**
 * @file lv_templ.h
 *
 */

#ifndef SH1107_H
#define SH1107_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif
#include "../lvgl_helpers.h"

/*********************
 *      DEFINES
 *********************/
#define SSD1322_DC       CONFIG_LV_DISP_PIN_DC
#define SSD1322_RST      CONFIG_LV_DISP_PIN_RST
#define SSD1322_USE_RST  CONFIG_LV_DISP_USE_RST
#define LV_HOR_RES_MAX   256
#define LV_VER_RES_MAX   64


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ssd1322_init(void);
void ssd1322_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void ssd1322_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area);
void ssd1322_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa);
void ssd1322_sleep_in(void);
void ssd1322_sleep_out(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*SH1107_H*/
