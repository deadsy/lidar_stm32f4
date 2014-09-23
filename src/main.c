//-----------------------------------------------------------------------------
/*



*/
//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "gpio.h"
#include "debounce.h"
#include "timers.h"
#include "stm32f4_regs.h"

#include "lidar.h"

USBD_HandleTypeDef hUSBDDevice;

//-----------------------------------------------------------------------------

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1);
}
#endif

void Error_Handler(void)
{
    while (1);
}

//-----------------------------------------------------------------------------

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    // Enable Power Control clock
    __PWR_CLK_ENABLE();

    // The voltage scaling allows optimizing the power consumption when the device is
    // clocked below the maximum system frequency, to update the voltage scaling value
    // regarding system frequency refer to product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Enable HSE Oscillator and activate PLL with HSE as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    HAL_PWREx_ActivateOverDrive();

    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

//-----------------------------------------------------------------------------

void debounce_on_handler(uint32_t bits)
{
    if (bits & (1 << PUSH_BUTTON_BIT)) {
        gpio_set(LED_RED);
    }
}

void debounce_off_handler(uint32_t bits)
{
    if (bits & (1 << PUSH_BUTTON_BIT)) {
        gpio_clr(LED_RED);
    }
}

//-----------------------------------------------------------------------------

static void lcd_init(void)
{
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
}

//-----------------------------------------------------------------------------

void serial_write(uint8_t data);
uint8_t serial_read(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    lcd_init();
    gpio_init();
    timers_init();
    debounce_init();

    USART_t *sio = usart_init(0);
    PWM_t *pwm = pwm_init(0, 10000);
    LIDAR_t *lidar = lidar_init(0, sio, pwm);


    BSP_LCD_DisplayStringAtLine(0, (uint8_t *)"Hello LIDAR");

    USBD_Init(&hUSBDDevice, &VCP_Desc, 0);
    USBD_RegisterClass(&hUSBDDevice, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUSBDDevice, &USBD_CDC_fops);
    USBD_Start(&hUSBDDevice);

    // Delay any output to serial until the USB CDC port is working.
    HAL_Delay(1500);

    printf("\r\n");
    display_exceptions();


    while (1) {

        lidar_run(lidar);

        uint8_t x = serial_read();
        if (x != 0xff) {
             usart_tx(sio, x);
             while(usart_test_rx(sio) == 0);
             serial_write(usart_rx(sio));
        }
    }

    return 0;
}

//-----------------------------------------------------------------------------
