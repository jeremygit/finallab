/*!
 *  @file LEDs.c
 *
 *  @date Created on: 10 Apr 2017
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 */

/*!
**  @addtogroup LEDs_module LEDs module documentation
**  @{
*/
/* MODULE LEDs */

// Information regarding the memory map for MK70F12 processor.
#include "MK70F12.h"
#include "LEDs.h"
#include "CPU.h"

bool LEDs_Init(void)
{
  // Enable
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Set to GPIO, which is alternative 1
  PORTA_PCR11 = PORT_PCR_MUX(1);
  PORTA_PCR28 = PORT_PCR_MUX(1);
  PORTA_PCR29 = PORT_PCR_MUX(1);
  PORTA_PCR10 = PORT_PCR_MUX(1);

  //drive strength
  //PORTA_PCR11 |= PORT_PCR_DSE_MASK;

  // Clear data registers
  LEDs_Off(LED_ORANGE);
  LEDs_Off(LED_YELLOW);
  LEDs_Off(LED_GREEN);
  LEDs_Off(LED_BLUE);

  // Set the direction of GPIOA as output
  GPIOA_PDDR |= LED_ORANGE;
  GPIOA_PDDR |= LED_YELLOW;
  GPIOA_PDDR |= LED_GREEN;
  GPIOA_PDDR |= LED_BLUE;

}

void LEDs_On(const TLED color)
{
  GPIOA_PCOR |= color;
}

void LEDs_Off(const TLED color)
{
  GPIOA_PSOR |= color;
}

void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR |= color;
}

/*!
** @}
*/
