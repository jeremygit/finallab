/* !
 * @file Threads.h
 *
 *  @brief A module to aggregate and manage the Threads for the RTOS
 *
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 *
 *  @date Created on: 1 Jun 2017, Last updated 7 Jun 2017
 */

#ifndef THREADS_H
#define THREADS_H

// RTOS
#include "OS.h"

/** Common thread sizes for convenience */
#define THREADS_STACK_SIZE_256 256
#define THREADS_STACK_SIZE_512 512
#define THREADS_STACK_SIZE_1024 1024

#define THREADS_STACK_SIZE_SML 128
#define THREADS_STACK_SIZE_MED 256
#define THREADS_STACK_SIZE_LRG 512

/** Data structure to compose threads used in the RTOS */
typedef struct Thread
{
  void (*thread)(void*);
  void *pData;
  void *pStack;
  uint8_t priority;
} Thread_t;

/** Exposes threads we want to use */
extern Thread_t Threads_Init;
extern Thread_t Threads_Main;
extern Thread_t Threads_RTC;
extern Thread_t Threads_UARTRx;
extern Thread_t Threads_UARTTx;
extern Thread_t Threads_PIT;
extern Thread_t Threads_AccelEvent;
extern Thread_t Threads_AccelReadComplete;
extern Thread_t Threads_AccelHandleI2C;
extern Thread_t Threads_FTMChannel0OutputEvent;

#endif /* THREADS_H */
