
#include "HAL_GPIO.h"
#include <stdint.h>

static void config_pin(GPIO_TypeDef *gpio, uint32_t pin_Number,
                       uint32_t mode_type) {

  if (pin_Number > 7) {

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

  else {

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
  if (pin_Number > 7) {
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
  else {
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
