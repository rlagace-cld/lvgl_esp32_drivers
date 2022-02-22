/**
 * @file ssd1322.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ssd1322.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "SSD1322"

#define SSD1322_CMD_ENABLE_GRAYSCALE_TABLE              0x00
#define SSD1322_CMD_SET_COLUMN_START_END_ADDRESS        0x15
#define SSD1322_CMD_WRITE_RAM                           0x5c
#define SSD1322_CMD_READ_RAM                            0x5d
#define SSD1322_CMD_SET_ROW_ADDRESS                     0x75
#define SSD1322_CMD_SETUP                               0xa0
#define SSD1322_SETUP_HORIZONTAL_ADDRESS_INCREMENT      0x00
#define SSD1322_SETUP_VERTICAL_ADDRESS_INCREMENT        0x01
#define SSD1322_SETUP_DISABLE_COLUMN_ADDRESS_REMAP      0x00
#define SSD1322_SETUP_ENABLE_COLUMN_ADDRESS_REMAP       0x02
#define SSD1322_SETUP_DISABLE_NIBBLE_REMAP              0x00
#define SSD1322_SETUP_ENABLE_NIBBLE_REMAP               0x04
#define SSD1322_SETUP_SCAN_INCREMENT                    0x00
#define SSD1322_SETUP_SCAN_DECREMENT                    0x10
#define SSD1322_SETUP_DISABLE_COM_SPLIT_ODD_EVEN        0x00
#define SSD1322_SETUP_ENABLE_COM_SPLIT_ODD_EVEN         0x20
#define SSD1322_SETUP_DISABLE_DUAL_COM_MODE             0x01
#define SSD1322_SETUP_ENABLE_DUAL_COM_MODE              0x11
#define SSD1322_CMD_SET_DISPLAY_START_LINE              0xa1
#define SSD1322_CMD_SET_DISPLAY_OFFSET                  0xa2
#define SSD1322_CMD_ENTIRE_DISPLAY_OFF                  0xa4
#define SSD1322_CMD_ENTIRE_DISPLAY_ON                   0xa5
#define SSD1322_CMD_NORMAL_DISPLAY                      0xa6
#define SSD1322_CMD_INVERSE_DISPLAY                     0xa7
#define SSD1322_CMD_ENABLE_PARTIAL_DISPLAY              0xa8
#define SSD1322_CMD_EXIT_PARTIAL_DISPLAY                0xa9
#define SSD1322_CMD_FUNCTION_SELECT                     0xab
#define SSD1322_FUNCTION_SELECT_EXTERNAL_VDD            0x00
#define SSD1322_FUNCTION_SELECT_INTERNAL_VDD            0x01
#define SSD1322_CMD_SET_PHASE_LENGTH                    0xb1
#define SSD1322_PHASE_1_5_DCLKS                         0x02
#define SSD1322_PHASE_1_7_DCLKS                         0x03
#define SSD1322_PHASE_1_9_DCLKS                         0x04
#define SSD1322_PHASE_1_11_DCLKS                        0x05
#define SSD1322_PHASE_1_13_DCLKS                        0x06
#define SSD1322_PHASE_1_15_DCLKS                        0x07
#define SSD1322_PHASE_1_17_DCLKS                        0x08
#define SSD1322_PHASE_1_19_DCLKS                        0x09
#define SSD1322_PHASE_1_21_DCLKS                        0x0a
#define SSD1322_PHASE_1_23_DCLKS                        0x0b
#define SSD1322_PHASE_1_25_DCLKS                        0x0c
#define SSD1322_PHASE_1_27_DCLKS                        0x0d
#define SSD1322_PHASE_1_29_DCLKS                        0x0e
#define SSD1322_PHASE_1_31_DCLKS                        0x0f
#define SSD1322_PHASE_2_3_DCLKS                         (0x03<<4)
#define SSD1322_PHASE_2_4_DCLKS                         (0x04<<4)
#define SSD1322_PHASE_2_5_DCLKS                         (0x05<<4)
#define SSD1322_PHASE_2_6_DCLKS                         (0x06<<4)
#define SSD1322_PHASE_2_7_DCLKS                         (0x07<<4)
#define SSD1322_PHASE_2_8_DCLKS                         (0x08<<4)
#define SSD1322_PHASE_2_9_DCLKS                         (0x09<<4)
#define SSD1322_PHASE_2_10_DCLKS                        (0x0a<<4)
#define SSD1322_PHASE_2_11_DCLKS                        (0x0b<<4)
#define SSD1322_PHASE_2_12_DCLKS                        (0x0c<<4)
#define SSD1322_PHASE_2_13_DCLKS                        (0x0d<<4)
#define SSD1322_PHASE_2_14_DCLKS                        (0x0e<<4)
#define SSD1322_PHASE_2_15_DCLKS                        (0x0f<<4)
#define SSD1322_CMD_SET_CLOCK_DIVIDER_OSC_FREQ          0xb3
#define SSD1322_CLOCK_DIVIDE_BY_1                       0x00
#define SSD1322_CLOCK_DIVIDE_BY_2                       0x01
#define SSD1322_CLOCK_DIVIDE_BY_4                       0x02
#define SSD1322_CLOCK_DIVIDE_BY_8                       0x03
#define SSD1322_CLOCK_DIVIDE_BY_16                      0x04
#define SSD1322_CLOCK_DIVIDE_BY_32                      0x05
#define SSD1322_CLOCK_DIVIDE_BY_64                      0x06
#define SSD1322_CLOCK_DIVIDE_BY_128                     0x07
#define SSD1322_CLOCK_DIVIDE_BY_256                     0x08
#define SSD1322_CLOCK_DIVIDE_BY_512                     0x09
#define SSD1322_CLOCK_DIVIDE_BY_1024                    0x0a
#define SSD1322_OSC_FREQ_NORMAL_MINUS_13                0x00
#define SSD1322_OSC_FREQ_NORMAL_MINUS_12                0x10
#define SSD1322_OSC_FREQ_NORMAL_MINUS_11                0x20
#define SSD1322_OSC_FREQ_NORMAL_MINUS_10                0x30
#define SSD1322_OSC_FREQ_NORMAL_MINUS_9                 0x40
#define SSD1322_OSC_FREQ_NORMAL_MINUS_7                 0x50
#define SSD1322_OSC_FREQ_NORMAL_MINUS_6                 0x60
#define SSD1322_OSC_FREQ_NORMAL_MINUS_5                 0x70
#define SSD1322_OSC_FREQ_NORMAL_MINUS_4                 0x80
#define SSD1322_OSC_FREQ_NORMAL_MINUS_3                 0x90
#define SSD1322_OSC_FREQ_NORMAL_MINUS_2                 0xa0
#define SSD1322_OSC_FREQ_NORMAL_MINUS_1                 0xb0
#define SSD1322_OSC_FREQ_NORMAL                         0xc0
#define SSD1322_OSC_FREQ_NORMAL_PLUS_1                  0xd0
#define SSD1322_OSC_FREQ_NORMAL_PLUS_2                  0xe0
#define SSD1322_OSC_FREQ_NORMAL_PLUS_3                  0xf0
#define SSD1322_CMD_DISPLAY_ENHANCEMENT_A               0xb4
#define SSD1322_A_EXTERNAL_VSL                          0xa0
#define SSD1322_A_INTERNAL_VSL                          0xa2
#define SSD1322_A_ENHANCED_LOW_GS_DISPLAY_QUALITY       0xfd
#define SSD1322_A_NORMAL_QUALITY                        0xb5
#define SSD1322_CMD_SET_GPIO                            0xb5
#define SSD1322_GPIO_0_HIZ_INPUT_DISABLED               0x00
#define SSD1322_GPIO_0_HIZ_INPUT_ENABLED                0x01
#define SSD1322_GPIO_0_OUTPUT_LOW                       0x02
#define SSD1322_GPIO_0_OUTPUT_HIGH                      0x03
#define SSD1322_GPIO_1_HIZ_INPUT_DISABLED               0x00
#define SSD1322_GPIO_1_HIZ_INPUT_ENABLED                0x04
#define SSD1322_GPIO_1_OUTPUT_LOW                       0x08
#define SSD1322_GPIO_1_OUTPUT_HIGH                      0x0c
#define SSD1322_CMD_SET_SECOND_PRECHARGE_PERIOD         0xb6
#define SSD1322_SECOND_PRECHARGE_PERIOD_MASK            0x0f
#define SSD1322_CMD_SET_GRAY_SCALE_TABLE                0xb8
#define SSD1322_CMD_SELECT_DEFAULT_LINEAR_GRAY_SCALE    0xb9
#define SSD1322_CMD_SET_PRECHARGE_VOLTAGE               0xbb
#define SSD1322_PRECHARGE_VOLTAGE_MASK                  0x1f
#define SSD1322_CMD_SET_VCOMH                           0xbe
#define SSD1322_VCOMH_MASK                              0x0f
#define SSD1322_CMD_SET_CONTRAST_CURRENT                0xc1
#define SSD1322_CMD_MASTER_CONTRAST_CURRENT_CONTROL     0xc7
#define SSD1322_CMD_SET_MUX_RATIO                       0xca
#define SSD1322_CMD_DISPLAY_ENHANCEMENT_B               0xd1
#define SSD1322_B_NORMAL                                0xa2
#define SSD1322_B2                                      0x20
#define SSD1322_CMD_SET_COMMAND_LOCK                    0xfd
#define SSD1322_UNLOCK_INTERFACE                        0x12
#define SSD1322_LOCK_INTERFACE                          0x16
#define SSD1322_CMD_DISPLAY_OFF                         0xae
#define SSD1322_CMD_DISPLAY_ON                          0xaf

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ssd1322_send_cmd(uint8_t cmd);
static void ssd1322_send_data(void * data, uint16_t length);
static void ssd1322_send_color(void * data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ssd1322_init(void)
{
    // Use Double Bytes Commands if necessary, but not Command+Data
    lcd_init_cmd_t init_cmds[]={
        {SSD1322_CMD_SET_COMMAND_LOCK, {SSD1322_UNLOCK_INTERFACE}, 1},  // Unlock cpu interface
/*        {SSD1322_CMD_SET_COLUMN_START_END_ADDRESS, {0x1c, 0x5b}, 2},    // Set display start line
        {SSD1322_CMD_WRITE_RAM, {0}, 0},                                // Write RAM
        {SSD1322_CMD_READ_RAM, {0}, 0},                                 // Read RAM
        {SSD1322_CMD_SET_ROW_ADDRESS, {0x00, 0x3f}, 2},                 // Set row address*/
        {SSD1322_CMD_SETUP, {
            SSD1322_SETUP_HORIZONTAL_ADDRESS_INCREMENT |
            SSD1322_SETUP_ENABLE_COLUMN_ADDRESS_REMAP |
            SSD1322_SETUP_ENABLE_NIBBLE_REMAP |
            SSD1322_SETUP_SCAN_INCREMENT |
            SSD1322_SETUP_DISABLE_COM_SPLIT_ODD_EVEN,
            SSD1322_SETUP_DISABLE_DUAL_COM_MODE
            }, 2},                                                      // Set Re-map and DUAL COM Line mode
        {SSD1322_CMD_SET_DISPLAY_START_LINE, {0}, 1},                   // Set display start line
        {SSD1322_CMD_SET_DISPLAY_OFFSET, {0}, 1},                       // Set display offset
        {SSD1322_CMD_NORMAL_DISPLAY, {0}, 0},                           // Set display mode normal
        {SSD1322_CMD_EXIT_PARTIAL_DISPLAY, {0}, 0},                     // Exit partial mode
        {SSD1322_CMD_FUNCTION_SELECT, {
            SSD1322_FUNCTION_SELECT_INTERNAL_VDD
            }, 1},                                                      // Select internal VDD
        {SSD1322_CMD_SET_PHASE_LENGTH, {
            SSD1322_PHASE_1_17_DCLKS |
            SSD1322_PHASE_2_14_DCLKS
            }, 0},                                                      // Set phases length
        {SSD1322_CMD_SET_CLOCK_DIVIDER_OSC_FREQ, {
            SSD1322_CLOCK_DIVIDE_BY_2 |
            SSD1322_OSC_FREQ_NORMAL_MINUS_3
            }, 1},                                                      // Set oscillator and frequency
        {SSD1322_CMD_DISPLAY_ENHANCEMENT_A, {
            SSD1322_A_EXTERNAL_VSL,
            SSD1322_A_NORMAL_QUALITY
            }, 2},
        {SSD1322_CMD_SET_GPIO, {
            SSD1322_GPIO_0_HIZ_INPUT_DISABLED |
            SSD1322_GPIO_1_HIZ_INPUT_DISABLED
            }, 1},
        {SSD1322_CMD_SET_SECOND_PRECHARGE_PERIOD, {0x0f}, 1},
        {SSD1322_CMD_SELECT_DEFAULT_LINEAR_GRAY_SCALE, {0}, 0},
        {SSD1322_CMD_SET_PRECHARGE_VOLTAGE, {0x0f}, 1},
        {SSD1322_CMD_SET_VCOMH, {0x07 & SSD1322_VCOMH_MASK}, 1},
        {SSD1322_CMD_DISPLAY_ENHANCEMENT_B, {
            SSD1322_B_NORMAL,
            SSD1322_B2
        }, 2},
        {SSD1322_CMD_SET_CONTRAST_CURRENT, {0xdf}, 1},
        {SSD1322_CMD_MASTER_CONTRAST_CURRENT_CONTROL, {0x0f}, 1},
        {SSD1322_CMD_SET_MUX_RATIO, {0x3f}, 1},
        {SSD1322_CMD_DISPLAY_ON, {0}, 0},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    gpio_pad_select_gpio(SSD1322_DC);
    gpio_set_direction(SSD1322_DC, GPIO_MODE_OUTPUT);

#if SSD1322_USE_RST
    gpio_pad_select_gpio(SSD1322_RST);
    gpio_set_direction(SSD1322_RST, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(SSD1322_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(SSD1322_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
#endif

    //Send all the commands
    uint16_t cmd = 0;
    while (init_cmds[cmd].databytes!=0xff) {
        ESP_LOGD(TAG, "cmd=%x", init_cmds[cmd].cmd);
        if (init_cmds[cmd].databytes & 0x1f)
        {
            ESP_LOG_BUFFER_HEXDUMP(TAG, init_cmds[cmd].data, init_cmds[cmd].databytes, ESP_LOG_DEBUG);
        }
        ssd1322_send_cmd(init_cmds[cmd].cmd);
        ssd1322_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes&0x1F);
        if (init_cmds[cmd].databytes & 0x80) {
        vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
}

void ssd1322_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
        lv_color_t color, lv_opa_t opa)
{
    /* buf_w will be ignored, the configured CONFIG_LV_DISPLAY_HEIGHT and _WIDTH,
        and CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE and _PORTRAIT will be used. */
    uint16_t byte_index = 0;
    uint8_t  shift;

#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE
    byte_index = (x >> 1) + (y * (disp_drv->hor_res >> 1));
    shift = (x & 0x01) ? 0 : 4;
#elif defined CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT
    byte_index = x + ((y >> 1) * disp_drv->ver_res);
    shift = (y & 0x01) ? 4 : 0;
#endif
    uint8_t c = lv_color_brightness(color);
    if (c != 0)
    {
        ESP_LOGD(TAG, "set_px_cb x=%d y=%d c=%d buf=%p byte_index=%u shift=%u", x, y, c, buf, byte_index, shift);
    }
    buf[byte_index] = (buf[byte_index] & ~(0x0f << shift)) | ((c >> 4) << shift);
}

void ssd1322_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    ESP_LOGD(TAG, "flush area.x1=%d area.x2=%d area.y1=%d area.y2=%d color_map=%p", area->x1, area->x2, area->y1, area->y2, color_map);
    uint8_t columns[2];
    uint8_t rows[2];
    uint32_t size = 0;
    void *ptr;

#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE
    rows[0] = area->y1;
    rows[1] = area->y2;
#else
    rows[0] = area->x1;
    rows[1] = area->x2;
#endif
    //columns[0] = (area->x1 >> 1) + 0x1c;
    //columns[1] = (area->x2 >> 2) + 0x1c;
    //size = columns[1] - columns[0] + 1;
    columns[0] = 0x1c;
    columns[1] = 0x5b;
    rows[0] = 0;
    rows[1] = 0x3f;
    size = 128;
    ssd1322_send_cmd(SSD1322_CMD_SET_COLUMN_START_END_ADDRESS);     // Set Column Start and End Address
    ssd1322_send_data(columns, 2);
    ssd1322_send_cmd(SSD1322_CMD_SET_ROW_ADDRESS);
    ssd1322_send_data(rows, 2);
    ESP_LOGD(TAG, "columns[0]=%u columns[1]=%u rows[0]=%u rows[1]=%u color_map=%p size=%u", columns[0], columns[1], rows[0], rows[1], color_map, size);
    ssd1322_send_cmd(SSD1322_CMD_WRITE_RAM);
    for(int i = rows[0]; i < (rows[1] + 1); i++)
    {
        ESP_LOGD(TAG, "i=%d", i);
#if defined CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE
        ptr = color_map + (i - rows[0]) * size;
#else
        ptr = color_map + (i - rows[0]) * size;
#endif
        ssd1322_send_data( (void *) ptr, size);
        ESP_LOG_BUFFER_HEXDUMP(TAG, ptr, size, ESP_LOG_DEBUG);
    }
/*    columns[0] = 0;
    columns[1] = drv->hor_res >> 1;
    rows[0] = 0;
    rows[1] = drv->ver_res;
    ssd1322_send_cmd(SSD1322_CMD_SET_COLUMN_START_END_ADDRESS);     // Set Column Start and End Address
    ssd1322_send_data(columns, 2);
    ssd1322_send_cmd(SSD1322_CMD_SET_ROW_ADDRESS);
    ssd1322_send_data(rows, 2);*/
    lv_disp_flush_ready(drv);
}

void ssd1322_rounder(lv_disp_drv_t * disp_drv, lv_area_t *area)
{
    ESP_LOGD(TAG, "rounder before area.x1=%d area.x2=%d area.y1=%d area.y2=%d", area->x1, area->x2, area->y1, area->y2);
/*    area->x1 = (area->x1 & (~1));
    area->x2 = (area->x2 & (~1)) + 1;*/
    area->x1 = area->y1 = 0;
    area->x2 = disp_drv->hor_res - 1;
    area->y2 = disp_drv->ver_res - 1;
    ESP_LOGD(TAG, "rounder after  area.x1=%d area.x2=%d area.y1=%d area.y2=%d", area->x1, area->x2, area->y1, area->y2);
}

void ssd1322_sleep_in()
{
    ESP_LOGI(TAG, "sleep_in");
    ssd1322_send_cmd(SSD1322_CMD_DISPLAY_OFF);
}

void ssd1322_sleep_out()
{
    ESP_LOGI(TAG, "sleep_out");
    ssd1322_send_cmd(SSD1322_CMD_DISPLAY_ON);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


static void ssd1322_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(SSD1322_DC, 0);	 /*Command mode*/
    disp_spi_send_data(&cmd, 1);
}

static void ssd1322_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(SSD1322_DC, 1);	 /*Data mode*/
    disp_spi_send_data(data, length);
}

static void ssd1322_send_color(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(SSD1322_DC, 1);   /*Data mode*/
    disp_spi_send_colors(data, length);
}
