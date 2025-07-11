#include "RTE_Components.h"
#include CMSIS_device_header
#include "HAL_GPIO.h"
#include "stm32f10x.h"

int main() {
  // // initialize clock at gpio c
  // RCC->APB2ENR |= 1 << 4;

  // // configure pin3 on gpio c
  // GPIOC->CRH |= (1 << 20 | 1 << 21);
  // GPIOC->CRH &= ~(1 << 22 | 1 << 23);
  GPIO_TYPE my_gpio;
  my_gpio.gpio = GPIOC;
  my_gpio.mode = OUTPUT_MODE;
  my_gpio.mode_type = OUTPUT_GEN_PURPOSE;
  my_gpio.speed = OUTPUT_50MHZ;

  gpio_init(my_gpio);

  while (1) {
    // GPIOC->BSRR = 1 << 13; // pin high
    // for (int i = 0; i <= 500000; i++);
    // GPIOC->BSRR = 1 << (13 + 16); // pin low
    //  for (int i = 0; i <= 500000; i++);
    gpio_toggle(GPIOC, 13);
    for (int i = 0; i <= 500000; i++)
      ;
  }
}
