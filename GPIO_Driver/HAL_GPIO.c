
#include "HAL_GPIO.h"
#include "stm32f10x.h"
#include <stdint.h>

static void config_pin(GPIO_TypeDef *gpio, uint32_t pin_Number,
                       uint32_t mode_type) {

  if (pin_Number > 7) { //control high register

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

  else {// control low registers

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

//*************************Configure pin speeds and modes*************************/

static void config_pin_speed(GPIO_TypeDef *gpio, uint32_t pin_Number,
                             uint32_t pin_speed, uint32_t mode) {
  if (pin_Number > 7) { //control high register
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
  else { //control low register
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

void gpio_write(GPIO_TypeDef *gpio, uint32_t pinNumber, uint8_t state)
{

if(state)
{
  gpio->BSRR = 1<<pinNumber;
}
else {
{
  gpio->BSRR = 1<<(pinNumber+16); 
}
}

}//gpio_write

void gpio_toggle(GPIO_TypeDef *gpio, uint32_t pinNummber)
{

  gpio->ODR ^= (1<<pinNummber);

}//gpio_toggle

void gpio_init(GPIO_TYPE gpio_type)
{
  if(gpio_type.gpio == GPIOA)
  ENABLE_GPIO_CLOCK_PORT_A;
 if(gpio_type.gpio == GPIOB)
  ENABLE_GPIO_CLOCK_PORT_B;
 if(gpio_type.gpio == GPIOC)
  ENABLE_GPIO_CLOCK_PORT_C;
 if(gpio_type.gpio == GPIOD)
  ENABLE_GPIO_CLOCK_PORT_D;

  config_pin(gpio_type.gpio, gpio_type.pin_Number, gpio_type.mode_type);
  config_pin_speed(gpio_type.gpio, gpio_type.pin_Number, gpio_type.speed, gpio_type.mode);

}
