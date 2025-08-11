
#include "HAL_GPIO.h"
#include "stm32f10x.h"
#include <stdint.h>

void ADC1_2_IRQHandler(int buff);

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
      AFIO->EXTICR[3] = AF[2] = {0, 0};
      IO_EXTICR4_EXTI12_PC;
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

//***************************USART FUNCTIONS ***************************** */

// void uart_init() {
//   RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;

//   GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_MODE9_1;
//   GPIOA->CRH &= ~(GPIO_CRH_MODE9_0);

//   USART1->BRR = 0x1d4c;

//   USART1->CR1 |= 1 << 2 | 1 << 3 | 1 << 13;

//   while (1) {

//     if (USART1->SR & USART_SR_RXNE) {

//       char temp[] = "Hello";
//       USART1->DR = temp[6];
//       while (!(USART1->SR & USART_SR_TC)) {
//         ;
//       }
//     }
//   }
// }

//**************************ADC Functions************************
//  */

void adc_init(uint32_t seq_len, uint16_t *samples, ADC_TypeDef *adc_num) {
  // change prescaler  for adc to not exceed 14Mhz
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
  // enable RCC clocks
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;

  // enabled DMA
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // set number of channels
  adc_num->SQR1 |= (seq_len - 1) << 20;

  adc_num->CR1 |= ADC_CR1_SCAN;
  adc_num->CR2 |= ADC_CR2_DMA;

  // dma settings
  DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
  DMA1_Channel1->CMAR = (uint32_t)samples;
  DMA1_Channel1->CNDTR = seq_len;
  DMA1_Channel1->CCR =
      DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
  DMA1_Channel1->CCR = DMA_CCR1_EN;
}

void adc_config(uint32_t chan_num, ADC_TypeDef *adc_num, uint32_t seq_len,
                uint32_t buff) {

  // enable end of conversion interrupt
  // adc_num->CR1 |= ADC_CR1_EOCIE;
  // enable that interrupt in nvic
  // NVIC_EnableIRQ(ADC1_2_IRQn);

  if (chan_num > 9) {
    adc_num->SMPR1 |= 1 << ((chan_num - 10) * 3 + 0) |
                      1 << ((chan_num - 10) * 3 + 1) |
                      1 << ((chan_num - 10) * 3 + 2);
  } else

  {
    adc_num->SMPR2 |= 1 << (chan_num * 3 + 0) | 1 << (chan_num * 3 + 1) |
                      1 << (chan_num * 3 + 2);
  }

  // set the sequence of the channels
  ADC1->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2;

  // dma setting

  // enable the adc for the first time and set to continous mode

  ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_CONT;

  for (int i = 0; i <= 5000000; i++) {
    ;
  }
  // turn on adc for the 2nd time as given in the data sheet
  ADC1->CR2 |= ADC_CR2_ADON;

  for (int i = 0; i <= 5000000; i++) {
    ;
  }
  ADC1->CR2 |= ADC_CR2_CAL;
  while (ADC1->CR2 == ~(ADC_CR2_CAL))
    ;

  // ADC1_2_IRQHandler(buff);
}

// void ADC1_2_IRQHandler(int buff) {
//   if (ADC1->SR & ADC_SR_EOC) {

//     buff = ADC1->DR;
//   }
// }