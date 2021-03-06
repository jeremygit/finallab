/*!
 *  @file FIFO.h
 *
 *  @brief Header file for FIFO private functions & FIFO structure definition
 *
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 *
 *  @date Created on: 29 Mar 2017, Last updated 01 April 2017
 *
 *  Hardware/Software configuration required to use the module : NA
 *
 */
/*!
**  @defgroup FIFO_module FIFO module documentation
**  @{
*/
/* MODULE FIFO */

#ifndef FIFO_H
#define FIFO_H

#include "types.h"
// RTOS
#include "OS.h"

/** Number of bytes that can fit in a FIFO */
#define FIFO_SIZE 256

/** FIFO data structure */
typedef struct
{				/*!< FIFO data structure */
  uint16_t Start;		/*!< The index of the position of the oldest data in the FIFO OR = GetPt */
  uint16_t End; 		/*!< The index of the next available empty position in the FIFO  = PutPt */
  //uint16_t volatile NbBytes;	/*!< The number of bytes currently stored in the FIFO */
  uint8_t Buffer[FIFO_SIZE];	/*!< The actual array of bytes to store the data */
  OS_ECB *BytesFreeSemaphore;	/*!< Counting semaphore so that calls FIFO put are blocked if no space free */
  OS_ECB *BytesUsedSemaphore;	/*!< Counting semaphore so that calls FIFO get are blocked if there are no bytes used (bytes in the buffer) */
} TFIFO;

/*! @brief Initialize the FIFO before first use.
 *
 *  @param FIFO A pointer to the FIFO that needs initializing.
 *  @return void
 */
void FIFO_Init(TFIFO * const FIFO);

/*! @brief Put one character into the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Put(TFIFO * const FIFO, const uint8_t data);

/*! @brief Get one character from the FIFO.
 *
 *  @param FIFO A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
void FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr);

#endif

/*!
** @}
*/

