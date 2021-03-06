/*!
 *  @file UART.c
 *
 *  @brief Definitions of the private UART functions
 *
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 *
 *  @date Created on: 29 Mar 2017, Last updated 01 April 2017
 *
 *  Hardware/Software configuration required to use the module : NA
 */
/*!
**  @addtogroup UART_module UART module documentation
**  @{
*/
/* MODULE UART */


#include "FIFO.h"
// Information regarding the memory map for MK70F12 processor.
#include "MK70F12.h"
#include "Cpu.h"
// New non-standard types
#include "types.h"
// Import the FIFO module.
// RTOS
#include "OS.h"

// Transfer FIFO Buffer
static TFIFO TxFIFO;
// Receive FIFO Buffer
static TFIFO RxFIFO;

// RTOS Semaphores to signal RX and TX occurances
static OS_ECB *UART_RXSemaphore;
static OS_ECB *UART_TXSemaphore;

// Interrupt for UART2
// IRQ 49 : 49 mod 32 = 17
// Non-IPR 1
// IPR 12

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{

  UART_RXSemaphore = OS_SemaphoreCreate(0);

  UART_TXSemaphore = OS_SemaphoreCreate(0);

  // Register keyword for sbr, brfa
  uint16_t sbr;
  uint16_t brfa;

  /* register setup to enable UART*/

  // Gate Control Register for UART2 and PORTE to enable pin routing
  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  /*! Pin Gate Control to enable multiplexing of pins of PORTE bit 16 and 17 to UART2_TX and UART_RX
   *  Search for PORT_PCR_MUX(x) in MK70F12.h for more explicit process
   *  In this case, the Alternate 3 is chosen in the Pin MUX control
   */
  PORTE_PCR16 = PORT_PCR_MUX(3);
  PORTE_PCR17 = PORT_PCR_MUX(3);

  // Disable the Rx and Tx of the UART
  UART2_C2 &= ~UART_C2_TE_MASK;
  UART2_C2 &= ~UART_C2_RE_MASK;
  UART2_C1 = 0x00;

  /*! Calculation of the System Baud Rate(SBR) and Baud Rate Fine Adjust(BRFA) - dependent upon input baud rate
   * These code are based on the formula provided in the K70 manual : UART Baud Rate = UART moduleclk / (16 * (sbr + brd [or aka BRFA/4])
   */
  brfa = ((moduleClk * 2) / baudRate) % 32;
  sbr = moduleClk / (16 * baudRate);
  //Set the respective values to their corresponding registers to generate a synchronized baud rate
  //UART_C4_BRFA(brfa);
  UART2_C4 |= (brfa & UART_C4_BRFA_MASK);
  UART2_BDH |= 0x1F & (sbr >> 8);
  UART2_BDL = sbr;

  // Enable the Rx and Tx of the UART
  UART2_C2 |= UART_C2_TE_MASK;
  UART2_C2 |= UART_C2_RE_MASK;

  FIFO_Init(&TxFIFO);
  FIFO_Init(&RxFIFO);

  // Clear pending interrupts
  NVICICPR1 = (1 << 17);
  // Enable Interrupts
  NVICISER1 = (1 << 17);

  // Enable Receive interrupts...
  UART2_C2 |= UART_C2_RIE_MASK;

  UART2_C2 |= UART_C2_TIE_MASK;

  //Initialization is successful
  return true;

}

void UART_InChar(uint8_t * const dataPtr)
{
  FIFO_Get(&RxFIFO, dataPtr);
}

void UART_OutChar(const uint8_t data)
{
  FIFO_Put(&TxFIFO, data);
}

void UART_Poll(void)
{
  // Depending on whether the Receive Data Register Full or Transmitted Data Register Empty (RDRF/TDRE)is set,
  // the data is either stored or retrieved.
  if (UART2_S1 & UART_S1_RDRF_MASK)
  {
    FIFO_Put(&RxFIFO, UART2_D);
  }
  if (UART2_S1 & UART_S1_TDRE_MASK)
  {
    FIFO_Get(&TxFIFO, (uint8_t *)&UART2_D);
  }
}

// TODO, potentially make one thread with arg specifying its function of Rx or Tx
void UART_RxThread(void *arg)
{
  OS_ERROR error;
  for (;;)
  {
    // Pause (back to main) after saving the byte
    (void)OS_SemaphoreWait(UART_RXSemaphore, 0);
    // Receive register was not full, so put the byte in the receive fifo

    // Wait until 256 bytes, but signal has byte (to start main packet handling)
    FIFO_Put(&RxFIFO, UART2_D);

    // Reenable Rx Interrupts
    UART2_C2 |= UART_C2_RIE_MASK;
  }
}

void UART_TxThread(void *arg)
{
  OS_ERROR error;
  for (;;)
  {
    // Pause thread for interrupt to signal
    (void)OS_SemaphoreWait(UART_TXSemaphore, 0);

    // Get a byte from the buffer
    FIFO_Get(&TxFIFO, (uint8_t *)&UART2_D);
    // Pause at the last if the last command made the buffer empty, and the loop went around

    // if there ar emore bytes to send, reenable
    UART2_C2 |= UART_C2_TIE_MASK;
  }
}

void __attribute__((interrupt)) UART2_ISR(void)
{
  OS_ISREnter();

  // Transmit
  // Transmit interrupts are enabled

  if (UART2_C2 & UART_C2_TIE_MASK)
  {
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      // signal sem for thread which checks if there is data
      (void) OS_SemaphoreSignal(UART_TXSemaphore);
      UART2_C2 &= ~UART_C2_TIE_MASK;
    }
  }

  // Receive
  if (UART2_C2 & UART_C2_RIE_MASK)
  {
    if (UART2_S1 & UART_S1_RDRF_MASK)
    {
      // Clear flag by reading UART_D or disable Rx interrupts
      UART2_C2 &= ~UART_C2_RIE_MASK;
      (void)OS_SemaphoreSignal(UART_RXSemaphore);
    }
  }


  OS_ISRExit();
}

/*
// ISR up to Lab4
void __attribute__((interrupt)) UART2_ISR(void)
{
  // Recieve
  if (UART2_C2 & UART_C2_RIE_MASK)
  {
    if (UART2_S1 & UART_S1_RDRF_MASK)
    {
      // Receive register was full, so put the byte in the recieve fifo
      FIFO_Put(&RxFIFO, UART2_D);
    }
  }

  // Transmit
  // Transmit interrupts are enabled
  if (UART2_C2 & UART_C2_TIE_MASK)
  {
    // And the transmit register is ready for something to put in it...
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      // Attempt to get the transfer-byte from the fifo...
      if (FIFO_Get(&TxFIFO, (uint8_t *)&UART2_D) == 0)
      {
	// Nothing to put (fifo empty)? disable transfer-ready interrupts
	UART2_C2 &= ~UART_C2_TIE_MASK;
      }
    }
  }

}
*/

/*!
** @}
*/
