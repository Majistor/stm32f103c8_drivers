
#include "HAL_GPIO.h"
#include "stm32f10x.h"
#include <stdint.h>

static void config_pin(GPIO_TypeDef *gpio, uint32_t pin_Number,
                       uint32_t mode_type) {

  if (pin_Number > 7) { // control high register

    switch (mode_type) {

      //******************OUTPUT AND INPUT MODE TYPRS***************** */
      // bcz inputs also have similar values

    case OUTPUT_GEN_PURPOSE | INPUT_ANALOG:
      gpio->CRH &= ~(1 << CNF_POS_BIT1 | 1 << CNF_POS_BIT2);
      break;
    case OUTPUT_OD | INPUT_FLOATING:
      gpio->CRH &= ~(1 << CNF_POS_BIT2);

      gpio->CRH |= (1 << CNF_POS_BIT1);
      break;

    case OUTPUT_ALT_FUNCTION | INPUT_PU_PD:
      gpio->CRH |= (1 << CNF_POS_BIT2);
      gpio->CRH &= ~(1 << CNF_POS_BIT1);
      break;

    case OUTPUT_ALT_FUNCTION_OD:
      gpio->CRH |= 1 << CNF_POS_BIT1 | 1 << CNF_POS_BIT2;
      break;

    } // switch
  } // if

  else { // control low registers

    switch (mode_type) {

    case OUTPUT_GEN_PURPOSE | INPUT_ANALOG:
      gpio->CRL &= ~(1 << CNF_POS_BIT1 | 1 << CNF_POS_BIT2);
      break;
    case OUTPUT_OD | INPUT_FLOATING:
      gpio->CRL &= ~(1 << CNF_POS_BIT2);
      gpio->CRL |= (1 << CNF_POS_BIT1);
      break;

    case OUTPUT_ALT_FUNCTION | INPUT_PU_PD:
      gpio->CRL |= (1 << CNF_POS_BIT2);
      gpio->CRL &= ~(1 << CNF_POS_BIT1);
      break;

    case OUTPUT_ALT_FUNCTION_OD:
      gpio->CRL |= 1 << CNF_POS_BIT1 | 1 << CNF_POS_BIT2;
      break;

    } // switch

  } // else
} // config_pin

//*************************Configure pin speeds and
// modes*************************/

static void config_pin_speed(GPIO_TypeDef *gpio, uint32_t pin_Number,
                             uint32_t pin_speed, uint32_t mode) {
  if (pin_Number > 7) { // control high register
    if (mode == INPUT_MODE) {
      gpio->CRH &= ~(1 << MODE_POS_BIT1 | 1 << MODE_POS_BIT2);
    } // if
    else {
      if (pin_speed == OUTPUT_10MHZ) {
        gpio->CRH &= ~(1 << MODE_POS_BIT2);
        gpio->CRH |= 1 << MODE_POS_BIT1;
      } // if

      if (pin_speed == OUTPUT_2MHZ) {

        gpio->CRH |= (1 << MODE_POS_BIT2);
        gpio->CRH &= ~(1 << MODE_POS_BIT1);

      } // if

      if (pin_speed == OUTPUT_50MHZ) {
        gpio->CRH |= (1 << MODE_POS_BIT1 | 1 << MODE_POS_BIT2);
      }

    } // else
  } // if
  else { // control low register
    if (mode == INPUT_MODE) {
      gpio->CRL &= ~(1 << MODE_POS_BIT1 | 1 << MODE_POS_BIT2);
    } // if
    else {
      if (pin_speed == OUTPUT_10MHZ) {
        gpio->CRL &= ~(1 << MODE_POS_BIT2);
        gpio->CRL |= 1 << MODE_POS_BIT1;
      } // if

      if (pin_speed == OUTPUT_2MHZ) {

        gpio->CRL |= (1 << MODE_POS_BIT2);
        gpio->CRL &= ~(1 << MODE_POS_BIT1);

      } // if

      if (pin_speed == OUTPUT_50MHZ) {
        gpio->CRL |= (1 << MODE_POS_BIT1 | 1 << MODE_POS_BIT2);
      }

    } // else
  } // else

} // config_pin_speed

void gpio_write(GPIO_TypeDef *gpio, uint32_t pin_Number, uint8_t state) {

  if (state) {
    gpio->BSRR = 1 << pin_Number;
  } else {
    {
      gpio->BSRR = 1 << (pin_Number + 16);
    }
  }

} // gpio_write

void gpio_toggle(GPIO_TypeDef *gpio, uint32_t pin_Number) {

  gpio->ODR ^= (1 << pin_Number);

} // gpio_toggle

void gpio_init(GPIO_TYPE gpio_type) {
  if (gpio_type.gpio == GPIOA)
    ENABLE_GPIO_CLOCK_PORT_A;
  if (gpio_type.gpio == GPIOB)
    ENABLE_GPIO_CLOCK_PORT_B;
  if (gpio_type.gpio == GPIOC)
    ENABLE_GPIO_CLOCK_PORT_C;
  if (gpio_type.gpio == GPIOD)
    ENABLE_GPIO_CLOCK_PORT_D;

  config_pin(gpio_type.gpio, gpio_type.pin_Number, gpio_type.mode_type);
  config_pin_speed(gpio_type.gpio, gpio_type.pin_Number, gpio_type.speed,
                   gpio_type.mode);

} // gpio_init

//****************************INTERRUPT FUNCTIONS*********************** */

void config_gpio_interrupt(GPIO_TypeDef *gpio, uint32_t pin_Number,
                           edge_select edge) {

  ENABLE_GPIO_CLOCK_ALTERNATE_FUNCTION;
  if (gpio == GPIOA) {

    switch (pin_Number) {
    case 0:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;

    case 1:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;

    case 2:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
    case 3:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
    case 4:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
    case 5:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
    case 6:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
    case 7:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
    case 8:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
    case 9:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
    case 10:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
    case 11:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
    case 12:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
    case 13:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
    case 14:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
    case 15:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
    } // switch case
  } // if
  if (gpio == GPIOB) {

    switch (pin_Number) {
    case 0:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
    case 1:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
    case 2:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
    case 3:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
    case 4:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
    case 5:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
    case 6:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
    case 7:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
    case 8:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
    case 9:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
    case 10:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
    case 11:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
    case 12:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
    case 13:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
    case 14:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
    case 15:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
    } // switch case
  } // if
  if (gpio == GPIOC) {

    switch (pin_Number) {
    case 0:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
    case 1:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
    case 2:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
    case 3:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
    case 4:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
    case 5:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
    case 6:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
    case 7:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
    case 8:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
    case 9:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
    case 10:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
    case 11:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
    case 12:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
    case 13:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
    case 14:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
    case 15:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
    } // switch case
  } // if

  if (gpio == GPIOC) {

    switch (pin_Number) {
    case 0:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
    case 1:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
    case 2:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
    case 3:
      AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
    case 4:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
    case 5:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
    case 6:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
    case 7:
      AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
    case 8:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
    case 9:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
    case 10:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
    case 11:
      AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
    case 12:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
    case 13:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
    case 14:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
    case 15:
      AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
    } // switch case
  } // if

  if (edge == RISING_EDGE) {
    EXTI->RTSR |= 1 << pin_Number;
  }
  if (edge == FALLING_EDGE) {
    EXTI->FTSR |= 1 << pin_Number;
  }
  if (edge == RISING_FALLING_EDGE) {
    EXTI->RTSR |= 1 << pin_Number;
    EXTI->FTSR |= 1 << pin_Number;
  }
  //

} // config_gpio_interrupt

void enable_gpio_interrupt(uint32_t pin_Number, IRQn_Type irqNumber) {
  // enable interrupt in EXTI
  EXTI->IMR |= 1 << pin_Number;
  // enable interrupt in NVIC
  NVIC_EnableIRQ(irqNumber);

} // enable gpio interrupt

void clear_gpio_interrupt(uint32_t pin_Number) {
  EXTI->PR |= 1 << pin_Number;

} // clear gpio interrupt

//***************************GPIO FUNCTIONS ***************************** */

void uart_init(GPIO_TypeDef *gpio) {
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
}