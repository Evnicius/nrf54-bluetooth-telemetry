#ifndef HD44780_H
#define HD44780_H

#include <zephyr/device.h>
#include <stdint.h>

/* LCD configuration constants */
#define LCD_5x8_DOTS   0x00
#define LCD_5x10_DOTS  0x04

#if defined(CONFIG_SOC_SAM3X8E)
#define GPIO_NODE DT_NODELABEL(pioc)
#elif defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
#define GPIO_NODE  DT_NODELABEL(gpio1)
#define GPIO_NODE_2 DT_NODELABEL(gpio2)
#else
#error "Unsupported GPIO driver"
#endif

#if defined(CONFIG_SOC_SAM3X8E)
/* Define GPIO OUT to LCD */
#define GPIO_PIN_PC12_D0		12	/* PC12 - pin 51 */
#define GPIO_PIN_PC13_D1		13	/* PC13 - pin 50 */
#define GPIO_PIN_PC14_D2		14	/* PC14 - pin 49 */
#define GPIO_PIN_PC15_D3		15	/* PC15 - pin 48 */
#define GPIO_PIN_PC24_D4		24	/* PC24 - pin 6 */
#define GPIO_PIN_PC23_D5		23	/* PC23 - pin 7 */
#define GPIO_PIN_PC22_D6		22	/* PC22 - pin 8 */
#define GPIO_PIN_PC21_D7		21	/* PC21 - pin 9 */
#define GPIO_PIN_PC28_RS		28	/* PC28 - pin 3 */
#define GPIO_PIN_PC25_E			25	/* PC25 - pin 5 */
#define GPIO_NAME			"GPIO_"
#elif defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
/* nRF54L15 DK — all signals on gpio1.
 * P1.04/05/06/07 are reserved for UART20 (console TX/RX/RTS/CTS).
 * P1.08-10, P1.13-14 used for buttons/LEDs on the DK.
 * Free pins: P1.00-03, P1.11, P1.12. */
#define GPIO_PIN_PC12_D0		0	/* unused in 4-bit mode */
#define GPIO_PIN_PC13_D1		0	/* unused in 4-bit mode */
#define GPIO_PIN_PC14_D2		0	/* unused in 4-bit mode */
#define GPIO_PIN_PC15_D3		0	/* unused in 4-bit mode */
#define GPIO_PIN_PC24_D4		13	/* P1.13 (button0 on DK) */
#define GPIO_PIN_PC23_D5		7	/* P2.07 (gpio2 pin 7, LED2 on DK) */
#define GPIO_PIN_PC22_D6		11	/* P1.11 (moved from P1.04=UART TX) */
#define GPIO_PIN_PC21_D7		12	/* P1.12 (moved from P1.05=UART RX) */
#define GPIO_PIN_PC28_RS		10	/* P1.10 (LED1 on DK; lights when RS=HIGH) */
#define GPIO_PIN_PC25_E			9	/* P1.09 (P1.01 unusable on DK hw) */
#define GPIO_NAME			"GPIO_"
#endif

#if defined(CONFIG_BOARD_NRF54L15DK_NRF54L15_CPUAPP_NS)
extern const struct device *g_d5_dev;
#define D5_DEV g_d5_dev
#else
#define D5_DEV gpio_dev
#endif

/* Commands */
#define LCD_CLEAR_DISPLAY		0x01
#define LCD_RETURN_HOME			0x02
#define LCD_ENTRY_MODE_SET		0x04
#define LCD_DISPLAY_CONTROL		0x08
#define LCD_CURSOR_SHIFT		0x10
#define LCD_FUNCTION_SET		0x20
#define LCD_SET_CGRAM_ADDR		0x40
#define LCD_SET_DDRAM_ADDR		0x80

/* Display entry mode */
#define LCD_ENTRY_RIGHT			0x00
#define LCD_ENTRY_LEFT			0x02
#define LCD_ENTRY_SHIFT_INCREMENT	0x01
#define LCD_ENTRY_SHIFT_DECREMENT	0x00

/* Display on/off control */
#define LCD_DISPLAY_ON			0x04
#define LCD_DISPLAY_OFF			0x00
#define LCD_CURSOR_ON			0x02
#define LCD_CURSOR_OFF			0x00
#define LCD_BLINK_ON			0x01
#define LCD_BLINK_OFF			0x00

/* Display/cursor shift */
#define LCD_DISPLAY_MOVE		0x08
#define LCD_CURSOR_MOVE			0x00
#define LCD_MOVE_RIGHT			0x04
#define LCD_MOVE_LEFT			0x00

/* Function set */
#define LCD_8BIT_MODE			0x10
#define LCD_4BIT_MODE			0x00
#define LCD_2_LINE			0x08
#define LCD_1_LINE			0x00
#define LCD_5x10_DOTS			0x04
#define LCD_5x8_DOTS			0x00

/* Define some device constants */
#define LCD_WIDTH			16	/* Max char per line */
#define HIGH				1
#define LOW				0
/* in millisecond */
#define	ENABLE_DELAY			1

#define GPIO_PIN_WR(dev, pin, bit)						\
	do {									\
		if (gpio_pin_set_raw((dev), (pin), (bit))) {			\
			printk("Err set " GPIO_NAME "%d! %x\n", (pin), (bit));	\
		}								\
	} while (0)								\


#define GPIO_PIN_CFG(dev, pin, dir)						\
	do {									\
		if (gpio_pin_configure((dev), (pin), (dir))) {			\
			printk("Err cfg " GPIO_NAME "%d! %lx\n", (pin), (dir));	\
		}								\
	} while (0)

/* Initialization */
void pi_lcd_init(const struct device *gpio_dev,
                 uint8_t cols,
                 uint8_t rows,
                 uint8_t dotsize);

/* Basic control */
void pi_lcd_clear(const struct device *gpio_dev);
void pi_lcd_home(const struct device *gpio_dev);

/* Cursor */
void pi_lcd_set_cursor(const struct device *gpio_dev,
                       uint8_t col,
                       uint8_t row);

/* Display control */
void pi_lcd_display_on(const struct device *gpio_dev);
void pi_lcd_display_off(const struct device *gpio_dev);

void pi_lcd_cursor_on(const struct device *gpio_dev);
void pi_lcd_cursor_off(const struct device *gpio_dev);

void pi_lcd_blink_on(const struct device *gpio_dev);
void pi_lcd_blink_off(const struct device *gpio_dev);

/* Scrolling */
void pi_lcd_scroll_left(const struct device *gpio_dev);
void pi_lcd_scroll_right(const struct device *gpio_dev);

/* Text direction */
void pi_lcd_left_to_right(const struct device *gpio_dev);
void pi_lcd_right_to_left(const struct device *gpio_dev);

/* Auto scroll */
void pi_lcd_auto_scroll_left(const struct device *gpio_dev);
void pi_lcd_auto_scroll_right(const struct device *gpio_dev);

/* Write string */
void pi_lcd_string(const struct device *gpio_dev, char *msg);
void lcd_print_row(const struct device *dev, uint8_t row, const char *fmt, ...);

void lcd_print_vehspd_engspd_gear(const struct device *gpio_dev);

#endif