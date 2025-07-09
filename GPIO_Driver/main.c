#include "RTE_Components.h"
#include CMSIS_device_header
#include "stm32f10x.h"

int main()
{
    // initialize clock at gpio c
    RCC->APB2ENR |= 1 << 4;

    // configure pin3 on gpio c
    GPIOC->CRH |= (1 << 20 | 1 << 21);
    GPIOC->CRH &= ~(1 << 22 | 1 << 23);

    GPIOC->BSRR = 1 << 13; // pin high
    for (int i = 0; i <= 500000; i++)
        ;
    GPIOC->BSRR = 1 << (13 + 16); // pin low
}
