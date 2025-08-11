#include "RTE_Components.h"
#include <stdio.h>
#include CMSIS_device_header
#include "HAL_GPIO.h"
#include "stm32f10x.h"
void go_to_sleep(void);

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
  my_gpio.pin_Number = 13;

  gpio_init(my_gpio);

  config_gpio_interrupt(GPIOB, 4, RISING_EDGE);
  enable_gpio_interrupt(4, EXTI4_IRQn);

  //************************Power Controls************************ */

  if ((PWR->CSR) & (PWR_CSR_SBF)) // check standby flag
  {
    // clear power wake up flag

    PWR->CR |= PWR_CR_CWUF;

    // clear standby flag

    PWR->CR |= PWR_CR_CSBF;

    // printf("woke from standby");

  } // standby check "if"

  else {
    ;
    // printf("woke up from power cycle");
  }

  while (1) {
    // GPIOC->BSRR = 1 << 13; // pin high
    // for (int i = 0; i <= 500000; i++);
    // GPIOC->BSRR = 1 << (13 + 16); // pin low
    //  for (int i = 0; i <= 500000; i++);
    gpio_write(GPIOC, 13, 1);
    for (int i = 0; i <= 5000000; i++)
      ;
    gpio_write(GPIOC, 13, 0);
    for (int i = 0; i <= 5000000; i++)
      ;
    // go_to_sleep();
    // uart_init();
  }
}
void EXTI4_IRQ_Handler() { clear_gpio_interrupt(4); }
void go_to_sleep(void) {
  // enable the PWR control Clock
  RCC->APB1ENR |= (RCC_APB1ENR_PWREN);

  // set DEEPSLEEP bit of Cortex System control

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  // Select standbbby mode

  PWR->CR |= PWR_CR_PDDS;

  // clear wake up flag

  PWR->CR |= PWR_CR_CWUF;

  // enable wake up pin

  PWR->CSR |= PWR_CSR_EWUP;

  // request wait for interrupt

  __WFI();
}