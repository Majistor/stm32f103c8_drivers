#ifndef HAL_GPIO
#define HAL_GPIO

#include "RTE_Components.h"
#include "stm32f10x.h"
#include <stdint.h>

// STATE
#define HIGH 1
#define LOW 0

#define OUTPUT_MODE 0x01
#define INPUT_MODE 0x02

// input modes

#define INPUT_ANALOG ((uint32_t)0x00)   // Analog mode
#define INPUT_FLOATING ((uint32_t)0x01) // Floating input
#define INPUT_PU_PD ((uint32_t)0x02)    // input with pull up and pull down

// output modes
#define OUTPUT_GEN_PURPOSE ((uint32_t)0x00)     // general purpose output
#define OUTPUT_OD ((uint32_t)0x01)              // output open drain
#define OUTPUT_ALT_FUNCTION ((uint32_t)0x02)    // pish pull
#define OUTPUT_ALT_FUNCTION_OD ((uint32_t)0x03) // open drain

// pin_Number speeds
#define OUTPUT_10MHZ ((uint32_t)0x01) // 01: Output mode, max speed 10 MHz.
#define OUTPUT_2MHZ ((uint32_t)0x02)  // 10: Output mode, max speed 2 MHz.
#define OUTPUT_50MHZ ((uint32_t)0x03) // 11: Output mode, max speed 50 MHz

// enable clock
#define ENABLE_GPIO_CLOCK_ALTERNATE_FUNCTION (RCC->APB2ENR |= (1 << 0))
#define ENABLE_GPIO_CLOCK_PORT_A (RCC->APB2ENR |= (1 << 2))
#define ENABLE_GPIO_CLOCK_PORT_B (RCC->APB2ENR |= (1 << 3))
#define ENABLE_GPIO_CLOCK_PORT_C (RCC->APB2ENR |= (1 << 4))
#define ENABLE_GPIO_CLOCK_PORT_D (RCC->APB2ENR |= (1 << 5))

// mode bits of GPIOs

#define MODE_POS_BIT1 ((pin_Number % 8) * 4)
#define MODE_POS_BIT2 (((pin_Number % 8) * 4) + 1)

// CNF bits of GPIO

#define CNF_POS_BIT1 (((pin_Number % 8) * 4) + 2)
#define CNF_POS_BIT2 (((pin_Number % 8) * 4) + 3)

typedef struct {
  GPIO_TypeDef *gpio;
  uint32_t pin_Number;
  uint32_t mode;
  uint32_t mode_type;
  uint32_t pull;
  uint32_t speed;
  uint32_t alt_func;

} GPIO_TYPE;

typedef enum { RISING_EDGE, FALLING_EDGE, RISING_FALLING_EDGE } edge_select;

// Funnction Prototypes

//***************************************************************** */
//                    GPIO CONFIGURATION
static void config_pin(GPIO_TypeDef *gpio, uint32_t pin_Number,
                       uint32_t mode_type);
static void config_pin_speed(GPIO_TypeDef *gpio, uint32_t pin_Number,
                             uint32_t pin_speed, uint32_t mode);

//***************************GPIO USER FUNCTIONS********************** */

void gpio_write(GPIO_TypeDef *gpio, uint32_t pinNumber, uint8_t state);
void gpio_toggle(GPIO_TypeDef *gpio, uint32_t pinNummber);
void gpio_init(GPIO_TYPE gpio_type);

//*************************INTERRUPT FUNCTIONS********************* */

void config_gpio_interrupt(GPIO_TypeDef *gpio, uint32_t pin_Number,
                           edge_select edge);
void enable_gpio_interrupt(uint32_t pin_Number, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pin_Number);

//***************************UART FUNCTIONS***********************************
//*/

void uart_init();

//****************************ADC FUNCTIONS************************************
// */
void adc_config(uint32_t chan_num, ADC_TypeDef *adc_num, int buff);

#endif