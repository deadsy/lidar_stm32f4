//-----------------------------------------------------------------------------
/*

LCD Code

*/
//-----------------------------------------------------------------------------

#include <string.h>

#include "stm32f429i_discovery_lcd.h"

//-----------------------------------------------------------------------------

#define LCD_COLS 21
#define LCD_ROWS 20
#define LCD_SIZE (LCD_COLS * LCD_ROWS)

#define LAST_ROW_OFS (LCD_COLS * (LCD_ROWS - 1))

//-----------------------------------------------------------------------------

typedef struct lcd_driver {

    uint8_t screen[LCD_SIZE];
    int col;

} LCD_t;

static LCD_t lcd;

//-----------------------------------------------------------------------------

static void lcd_display_row(int row) {
    uint8_t tmp[LCD_COLS + 1];
    memcpy(tmp, &lcd.screen[row * LCD_COLS], LCD_COLS);
    tmp[LCD_COLS] = 0;
    BSP_LCD_DisplayStringAtLine(row, tmp);
}

//-----------------------------------------------------------------------------

void lcd_putchar(char c) {
    if (c == '\n') {
        memcpy(&lcd.screen[0], &lcd.screen[LCD_COLS], LCD_SIZE - LCD_COLS);
        memset(&lcd.screen[LAST_ROW_OFS], 0x20, LCD_COLS);
        for (int i = 0; i < LCD_ROWS; i ++) {
            lcd_display_row(i);
        }
        lcd.col = 0;
    } else {
        if (lcd.col < LCD_COLS) {
            lcd.screen[LAST_ROW_OFS + lcd.col] = c;
            lcd.col += 1;
            lcd_display_row(LCD_ROWS - 1);
        }
    }
}

// hook up stdio output to the lcd
int __io_putchar(int ch) {
    lcd_putchar(ch);
    return 0;
}

//-----------------------------------------------------------------------------

void lcd_init(void) {

    BSP_LCD_Init();

    BSP_LCD_LayerDefaultInit(0, (uint32_t)LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_RED);
    BSP_LCD_SetBackColor(LCD_COLOR_RED);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

//     BSP_LCD_LayerDefaultInit(1, (uint32_t)LCD_FRAME_BUFFER + 76800);
//     BSP_LCD_SelectLayer(1);
//     BSP_LCD_Clear(LCD_COLOR_RED);
//     BSP_LCD_SetBackColor(LCD_COLOR_RED);
//     BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    BSP_LCD_SetLayerVisible(0, ENABLE);
//     BSP_LCD_SetLayerVisible(1, ENABLE);

    BSP_LCD_DisplayOn();

    for (int i = 0; i < LCD_SIZE; i ++) {
        lcd.screen[i] = ' ';
    }
    lcd.col = 0;
}

//-----------------------------------------------------------------------------
