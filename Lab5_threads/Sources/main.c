/* ###################################################################
**     Filename    : main.c
**     Project     : Lab1
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 1.0
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */

// CPU mpdule - contains low level hardware initialization routines
#include "Cpu.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "OS.h"

// New non-standard types
#include "types.h"
// Include the Packet module
#include "Packet.h"
// Include the LEDs module
#include "LEDs.h"
// Include the Flash module
#include "Flash.h"
// FTM
#include "FTM.h"
// PIT
#include "PIT.h"
// RTC
#include "RTC.h"
// Accel for debug
#include "Accel.h"
//
#include "Threads.h"

/** To efficiently handle use of the Tower Version.*/
typedef union TVersion
{
  uint16_t bytes;
  struct structure {
    uint8_t major;
    uint8_t minor;
  } structure;
} TVersion;

/** To efficiently handle the Accel data acquisition mode used by the tower... Polling and Interrupts */
typedef enum
{
  TOWER_PROTOCOL_MODE_ASYNC,  // polling
  TOWER_PROTOCOL_MODE_SYNC, //
} TTowerProtocolMode;

#define TOWER_VERSION_MAJOR 1
#define TOWER_VERSION_MINOR 0
#define TOWER_MODE 1
#define TOWER_NUMBER 3445
#define TOWER_BAUD_RATE 115200

// Details about the Tower
static volatile int16union_t *NvTowerNumber; 	/*!< The Tower Number */
static volatile int16union_t *NvTowerMode; 	/*!< The Tower Number */
static volatile TVersion *NvTowerVersion;	/*!< The Tower Version */
static volatile TTowerProtocolMode *NvTowerProtocolMode; /*!< The Tower Protocol Mode */

// Accel Data Acquisition variables
/** The AccelData we want to write to */
static TAccelData AccelData;
/** Keep a copy of the prevous accel data to compare differenced with */
static TAccelData PrevAccelData;

/*! @brief Tower configuration.
 *   Checking whether the Tower Variables are saved in the flash memory.
 *   If not write them back into the flash memory.
 *  @return void
 */
static void TowerSetup(void);

/*! @brief Initiates the tower including its dependent modules and performs the tower startup protocol.
 *   This includes;
 *   LED init,
 *   Flash init &
 *   Packet init
 *   On success for all of the above, a orange led light will be lighten up.
 *  @return void
 */
static void TowerInit(void);

/*! @brief Handles the procedure for a received packet including executing the given Command and responding to an ACK.
 *   Clearing the MSB of the command parameter if an acknowledgment is requested.
 *   Then is sent to HandleCommand to deal with the received command.
 *  @return void
 */
static void HandlePacket(void);

/*! @brief Handles the control flow of the Tower in response to a received Command.
 *   interprets the received command and instructs tower to do the corresponding action.
 *  @return bool - TRUE if all Packets were successfully created and transmitted.
 */
static bool HandleCommand(void);

/*! @brief Assemble and transmit the 4 Packets in accordance with the protocol's "Start Up Values" Command.
 *   Startup, Tower number, Tower version and Tower mode packets are sent to PC on start up.
 *  @return bool - TRUE if all Packets were successfully created and transmitted.
 */
static bool SendStartupValues(void);

/*! @brief Assemble and transmit the Packet containing the "Tower Version" Command and Parameters in accordance with the protocol and attempts to transmit it.
 *  @return bool - TRUE if the Packet was created and transmitted successfully.
 */
static bool SendTowerVersion(void);

/*! @brief Assemble and transmit the Packet containing the "Tower Number" Command and Parameters in accordance with the protocol.
 *  @return bool - TRUE if the Packet was created and transmitted successfully.
 */
static bool SendTowerNumber(void);

/*! @brief Reads desired byte and prepares it into a packet to be sent back to the PC
 *   the offset is also returned with the desired byte
 *  @return bool - TRUE if Parameter 2 and 3 are zero and the Packet was created and transmitted successfully.
 */
static bool SendByteFromFlash(void);

/*! @brief Programs pre-designated byte into the flash memory
 *   after checking the input parameters from the PC are complies with the protocol - that is, true within their range or format.
 *  @return bool - TRUE if it is successfully programmed into the flash memory
 */
static bool ProgramByteToFlash(void);

/*! @brief Module that responds when receiving a Tower Mode command, and depending on the parameter 1 of the received packet,
 *  for get, it checks if the parameters are within the assigned range or value
 *  for set, it writes into flash memory before
 *  it is prepared into a packet and set back to the PC
 *  @return bool - TRUE if packet of tower mode is successfully packed.
 */
static bool SendTowerMode(void);

/*! @brief Module that sets the RTC time upon receiving the corresponding Set Time command
 *  checks whether the entered parameters are within in their reasonable range (hrs <= 23, etc)
 *  before it sets the time in the RTC
 *  @return bool - TRUE if the time is successfully set.
 */
static bool SetTime(void);

/*! @brief This procedure is performed when a valid packet is received to the Tower from the PC.
 *  That is, simply when a packet has not been corrupted in transit and the checksum passes.
 */
static void OnReceiveValidPacket(void);

/**************************************************************************************************/

/*! @brief Handles the changing of how the Tower protocol acquires data from the Accelerometer as either
 *  Synchronous or Asynchronous in response to a command from the PC.
 *  @return bool - TRUE if changed successfully.
 */
static bool HandleProtocolMode(void);

/*! @brief Performs the changing of the Towers protocol mode.
 *  @return bool - TRUE if changed successfully
 */
static void ConfigureTowerProtocol(void);

/*! @brief The Callback Function that is passed to the Accelerometer module that handles what happens
 *   after the Accelerometer has read X Y Z into the variable (i.e. send packet to PC)
 *  @param args, void pointer to support any data type the user passes to it.
 *  @return void
 */
static void AccelDataReadCompleteCallbackFunction(void *args);
/** The arguments that can be passed along with the above callback function */
static void *AccelDataReadCompleteCallbackArguments;

/*! @brief The Callback function that is passed to the Accelerometer module to handle the event
 *  of the Accelerometer indicating it has new data ready (i.e. we might want to initiate a read)
 *  @param args, void pointer to support any data type the user passes to it.
 *  @return void
 */
static void AccelDataReadyCallbackFunction(void *args);
/** The arguments that can be passed along with the above callback function */
static void *AccelDataReadyCallbackArguments;

/**************************************************************************************************/

/*! @brief Module that calls back to the main program when the FTM ISR is triggered
 *  Then toggles the current state of the blue LED
 *  @param args, void pointer to support any data type the user passes to it.
 */
static void FTMCallbackFunction(void *args);

/*! @brief Module that calls back to the main program when the PIT ISR is triggered
 *  Then toggles the current state of the green LED
 *  @param args, void pointer to support any data type the user passes to it.
 */
static void PITCallbackFunction(void *args);

/*! @brief Module that retrieves time from RTC when the RTC ISR is triggered
 *  and toggles the current state of the Yellow LED
 *  @param args, void pointer to support any data type the user passes to it.
 */
static void RTCCallbackFunction(void *args);

/**************************************************************************************************/
/** Init the Tower modules and perform Tower setup when it powers up */
static void InitThread(void *arg);
/** Main Thread handles the Packets from the PC after they're received and execute the corresponding functionality */
static void MainThread(void *arg);
/** RTC Thread that has the responsibility of sending a Packer containing the current time to the PC */
static void RTCSecondsChangeThread(void *arg);
/** PIT acquires data from the Accelerometer for Asyc/Polling protocol model and thus should run at 1 second intervals */
static void PITEventThread(void *arg);
/** Acceleromter thread to handle a signal from the hardware when it has data. It should initiate a read of the Accelerometer*/
static void AccelEventThread(void *arg);
/** Acceleromter thread to handle a signal from the hardware when it has data. It should initiate a read of the Accelerometer*/
static void AccelReadCompleteThread(void *arg);
/** FTM thread that handles Output compare interrupts/events */
static void FTMChannelOutputEventThread(void *arg);


/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  OS_ERROR error;

  PE_low_level_init();

  OS_Init(CPU_CORE_CLK_HZ, true);

  // Assign main created thread functions to the Thread data structures
  Threads_Init.thread = &InitThread;
  Threads_Main.thread = &MainThread;
  Threads_RTC.thread = &RTCSecondsChangeThread;
  Threads_PIT.thread = &PITEventThread;
  Threads_AccelEvent.thread = &AccelEventThread;
  Threads_AccelReadComplete.thread = &AccelReadCompleteThread;
  Threads_FTMChannel0OutputEvent.thread = &FTMChannelOutputEventThread;

  // Init Thread
  error = OS_ThreadCreate(Threads_Init.thread,
			  Threads_Init.pData,
			  Threads_Init.pStack,
			  Threads_Init.priority);

  // Main Thread
  error = OS_ThreadCreate(Threads_Main.thread,
			  Threads_Main.pData,
			  Threads_Main.pStack,
			  Threads_Main.priority);

  // RTC Thread
  error = OS_ThreadCreate(Threads_RTC.thread,
			  Threads_RTC.pData,
			  Threads_RTC.pStack,
			  Threads_RTC.priority);

  // UART RX Thread
  error = OS_ThreadCreate(Threads_UARTRx.thread,
			  Threads_UARTRx.pData,
			  Threads_UARTRx.pStack,
			  Threads_UARTRx.priority);

  // UART TX Thread
  error = OS_ThreadCreate(Threads_UARTTx.thread,
			  Threads_UARTTx.pData,
			  Threads_UARTTx.pStack,
			  Threads_UARTTx.priority);

  // Accel Handle I2C, after the I2C reads XYZ
  error = OS_ThreadCreate(Threads_AccelHandleI2C.thread,
			  Threads_AccelHandleI2C.pData,
			  Threads_AccelHandleI2C.pStack,
			  Threads_AccelHandleI2C.priority);

  // Accel Read Complete Thread, that send the packet
  error = OS_ThreadCreate(Threads_AccelReadComplete.thread,
			  Threads_AccelReadComplete.pData,
			  Threads_AccelReadComplete.pStack,
			  Threads_AccelReadComplete.priority);

  // Accel Event Thread, that triggers a read
  error = OS_ThreadCreate(Threads_AccelEvent.thread,
			  Threads_AccelEvent.pData,
			  Threads_AccelEvent.pStack,
			  Threads_AccelEvent.priority);

  // PIT Thread
  error = OS_ThreadCreate(Threads_PIT.thread,
			  Threads_PIT.pData,
			  Threads_PIT.pStack,
			  Threads_PIT.priority);

  // FTM Thread
  error = OS_ThreadCreate(Threads_FTMChannel0OutputEvent.thread,
			  Threads_FTMChannel0OutputEvent.pData,
			  Threads_FTMChannel0OutputEvent.pStack,
			  Threads_FTMChannel0OutputEvent.priority);
  // Never return...
  OS_Start();

  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/

}

/* END main */
/*!
** @}
*/

static void TowerInit(void)
{

  TAccelSetup accelSetup = {
      CPU_BUS_CLK_HZ,
      AccelDataReadyCallbackFunction,
      AccelDataReadyCallbackArguments,
      AccelDataReadCompleteCallbackFunction,
      AccelDataReadCompleteCallbackArguments
  };

  bool success =  LEDs_Init() &&
		  Flash_Init() &&
		  Packet_Init(TOWER_BAUD_RATE, CPU_BUS_CLK_HZ) &&
		  Accel_Init(&accelSetup) &&
		  FTM_Init() &&
		  PIT_Init(CPU_BUS_CLK_HZ, PITCallbackFunction, NULL) &&
		  RTC_Init(RTCCallbackFunction, NULL);
  if (success)
  {
    PIT_Set(1000000000, true); // start the PIT
    LEDs_On(LED_ORANGE); // turn on orange LED on successful initialization...
  }
}

static void TowerSetup(void)
{
  Flash_AllocateVar(&NvTowerNumber, sizeof(*NvTowerNumber));
  Flash_AllocateVar(&NvTowerMode, sizeof(*NvTowerMode));
  Flash_AllocateVar(&NvTowerVersion, sizeof(*NvTowerVersion));
  Flash_AllocateVar(&NvTowerProtocolMode, sizeof(*NvTowerProtocolMode));

  /* Check if Tower information variables are erased, if they are; the defined values are written to the flash memory. */
  if (_FH(NvTowerNumber) == FLASH_ERASED_HWORD)
  {
    Flash_Write16(NvTowerNumber, (uint16_t)TOWER_NUMBER);
  }

  if (_FH(NvTowerMode) == FLASH_ERASED_HWORD)
  {
    Flash_Write16(NvTowerMode, (uint16_t)TOWER_MODE);
  }

  if (_FH(NvTowerVersion) == FLASH_ERASED_HWORD)
  {
    TVersion towerVersion;
    towerVersion.structure.major = TOWER_VERSION_MAJOR;
    towerVersion.structure.minor = TOWER_VERSION_MINOR;
    Flash_Write16(NvTowerVersion, (uint16_t)towerVersion.bytes);
  }

  if (_FH(NvTowerProtocolMode) == FLASH_ERASED_HWORD)
  {
    Flash_Write16(NvTowerProtocolMode, TOWER_PROTOCOL_MODE_ASYNC); // set default to polling
  }

  ConfigureTowerProtocol();

  // Constructed the information into relevant packets and send back to the PC.
  SendStartupValues();
}

static void HandlePacket(void)
{
  // Attempt to carry out the requested Command and store the outcome in the success flag.
  bool success = HandleCommand();

  // Check whether an ACK request was received by examining the Command byte.
  if  (Packet_Command & PACKET_ACK_MASK)
  {

    // Unset the MSB of the received Command byte when the Command is not successfully handled.
    if (!success)
    {
      Packet_Command &= ~PACKET_ACK_MASK;
    }
    // Assemble the ACK packet with the same, received parameters.
    Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  }
}

static bool HandleCommand(void)
{
  switch(Packet_Command & ~PACKET_ACK_MASK)		// Interpret the received Command by ignoring the ACK bit.
  {
    case PACKET_CMD_STARTUP_VALUES:			// A Command for Startup Values was received.
      return SendStartupValues();			// Respond by attempting to send the Start Up Values.

    case PACKET_CMD_TOWER_VERSION:			// A Command for the Tower Version was received.
      return SendTowerVersion();			// Respond by attempting to send the Tower Version.

    case PACKET_CMD_TOWER_NUMBER:			// A Command to GET/SET the Tower Number was received.
      return SendTowerNumber();

    case PACKET_CMD_PROGRAM_FLASH_BYTE:			// A Command to SET a Byte in Flash
      return ProgramByteToFlash();      		// No Packet is sent back to the PC

    case PACKET_CMD_READ_FLASH_BYTE:      		// A Command to GET a byte out of Flash
      return SendByteFromFlash();      			// and send it in a Packet back to the PC

    case PACKET_CMD_TOWER_MODE:      			// A Command to GET/SET or set the Tower Mode value
      return SendTowerMode();

    case PACKET_CMD_SET_TIME:				//A command to set the time of the RTC
      return SetTime();

    case PACKET_CMD_PROTOCOL_MODE:			//A command to GET/SET the protocol mode for Accel
      return HandleProtocolMode();
  }
  return false;						// When an invalid Command is received.
}

// TODO : Reduce repeated code
static bool SendStartupValues(void)
{
  //Constructing the Startup packets
  Packet_Put(PACKET_CMD_STARTUP_VALUES, 0x00, 0x00, 0x00);
  Packet_Put(PACKET_CMD_TOWER_VERSION, 'v', NvTowerVersion->structure.major, NvTowerVersion->structure.minor);
  Packet_Put(PACKET_CMD_TOWER_NUMBER, 0x01, NvTowerNumber->s.Lo, NvTowerNumber->s.Hi);
  Packet_Put(PACKET_CMD_TOWER_MODE, 0x01, NvTowerMode->s.Lo, NvTowerMode->s.Hi);
  Packet_Put(PACKET_CMD_PROTOCOL_MODE, 0x01, *NvTowerProtocolMode, 0x00);
  return true;
}

static bool SendTowerVersion(void){
  // Parameter 1, 2 and 3 should be v, x and CR; respectively
  bool success = (Packet_Parameter1 == 0x76 &&
		  Packet_Parameter2 == 0x78 &&
		  Packet_Parameter3 == 0x0d);
  Packet_Put(PACKET_CMD_TOWER_VERSION, 'v', NvTowerVersion->structure.major, NvTowerVersion->structure.minor);
  return success;
}

static bool SendTowerNumber(void){
  bool success = false;
  switch(Packet_Parameter1)
  {
    case PACKET_MODE_GET:
      // ensure Parameter 2 and 3 were 0x00 for a GET command
      success = !Packet_Parameter23;
      break;
    case PACKET_MODE_SET:
      OS_DisableInterrupts();
      success = Flash_Write16(NvTowerNumber, Packet_Parameter23);
      OS_EnableInterrupts();
      break;
  }
  Packet_Put(PACKET_CMD_TOWER_NUMBER, 0x01, NvTowerNumber->s.Lo, NvTowerNumber->s.Hi);
  return success;
}

static bool SendByteFromFlash(void){
  bool success = false;
  uint8_t dataByte;
  success = !Packet_Parameter23 && Flash_ReadByte(FLASH_DATA_START + Packet_Parameter1, &dataByte);// attempt to read data from flash into dataByte, Parameter 2 and 3 should be zero from the PC
  Packet_Put(PACKET_CMD_READ_FLASH_BYTE, Packet_Parameter1, 0x00, dataByte); // Send back what was the offset, 0 and the read-byte
  return success;
}

static bool ProgramByteToFlash(void){
  bool success = false;
  // packet 2 should always be zero for a program byte command
  if (!Packet_Parameter2)
  {
    // make sure the first parameter complies with a program byte command
    if (Packet_Parameter1 >= 0x00 && Packet_Parameter1 < 0x08)
    {
      OS_DisableInterrupts();
      success = Flash_Write8((FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3);
      OS_EnableInterrupts();
    }
    else if (Packet_Parameter1 == 0x08)
    {
      OS_DisableInterrupts();
      success = Flash_Erase();
      OS_EnableInterrupts();
    }
  }
  // no packet is sent back to PC
  return success;
}

static bool SendTowerMode(void){
  bool success = false;
  switch (Packet_Parameter1)
  {
    case PACKET_MODE_GET:
      // ensure Parameter 2 and 3 are both 0x00 for a GET command
      success = !Packet_Parameter23;
      break;
    case PACKET_MODE_SET:
      OS_DisableInterrupts();
      success = Flash_Write16(NvTowerMode, Packet_Parameter23);
      OS_EnableInterrupts();
      break;
  }
  /*Prepare packet to be sent back to the PC*/
  Packet_Put(PACKET_CMD_TOWER_MODE, 0x01, NvTowerMode->s.Lo, NvTowerMode->s.Hi);
  return success;
}

static bool SetTime(void)
{
  //Check if the packet parameter is within reasonable ranges
  if (!RTC_IsValidTime(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3))
  {
    return false;
  }
  RTC_Set(Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  return true;
}

static bool HandleProtocolMode(void)
{

  bool success = false;
  switch (Packet_Parameter1)
  {
    case PACKET_MODE_GET:
      // ensure Parameter 2 and 3 are both 0x00 for a GET command
      success = !Packet_Parameter23;
      break;
    case PACKET_MODE_SET:
      // Ensure the user is trying to set an accetpable mode
      if (Packet_Parameter2 <= TOWER_PROTOCOL_MODE_SYNC)
      {
	OS_DisableInterrupts();
	success = Flash_Write16(NvTowerProtocolMode, Packet_Parameter2);
	OS_EnableInterrupts();
	// If the mode was changed in Flash successfully, configure the tower...
	if (success)
	{
	  ConfigureTowerProtocol();
	}
      }
      break;
  }
  /*Prepare packet to be sent back to the PC*/
  Packet_Put(PACKET_CMD_PROTOCOL_MODE, 0x01, *NvTowerProtocolMode, 0x00);
  return success;
}

static void ConfigureTowerProtocol(void)
{
  if (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_ASYNC)
  {
    //Polling
    PIT_Enable(true);
    Accel_SetMode(ACCEL_POLL);
  }
  else if (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_SYNC)
  {
    //Interrupts
    PIT_Enable(false);
    Accel_SetMode(ACCEL_INT);
  }
}

/*
** ###################################################################
**
**	Events:
**
** ###################################################################
*/
static void OnReceiveValidPacket(void)
{
  static const TFTMChannel ftmObj = {
    0, // channel number to configure
    // Number of periods/cycles to complete i.e. 24414 in this case since
    // the module uses Fixed Freq Clock and we want 1 second delay
    CPU_MCGFF_CLK_HZ_CONFIG_0,
    TIMER_FUNCTION_OUTPUT_COMPARE,
    TIMER_OUTPUT_HIGH,
    FTMCallbackFunction,
    NULL
  };

  // Seems to be necesarry
  OS_DisableInterrupts();

  FTM_Set(&ftmObj);
  LEDs_On(LED_BLUE);
  FTM_StartTimer(&ftmObj);

  OS_EnableInterrupts();
}

/*
** ###################################################################
**
**	Thread Functions:
**	...for FTM, PIT, RTC, Accel
**
** ###################################################################
*/

static void InitThread(void *arg)
{
  for (;;)
  {
    OS_DisableInterrupts();
    TowerInit();
    TowerSetup();
    OS_EnableInterrupts();
    OS_ThreadDelete(OS_PRIORITY_SELF);
  }
}

static void MainThread(void *arg)
{
  for (;;)
  {
    if (Packet_Get())
    {
      HandlePacket();
      OnReceiveValidPacket();
    }
  }
}

static void PITEventThread(void *arg)
{
  OS_ERROR error;
  for(;;)
  {
    (void)OS_SemaphoreWait(PIT_EventSemaphore, 0);
    Accel_ReadXYZ(&AccelData);
  }
}

static void AccelEventThread(void *arg)
{
  OS_ERROR error;
  for (;;)
  {
    (void)OS_SemaphoreWait(Accel_EventSemaphore, 0);
    Accel_ReadXYZ(&AccelData);
  }
}

static void AccelReadCompleteThread(void *arg)
{
  OS_ERROR error;

  bool canTransmitFlag = false;
  bool accelDataDidNotMatchFlag = false;

  for (;;)
  {
    (void)OS_SemaphoreWait(Accel_ReadCompleteSemaphore, 0);

    // Check whether the Previous and New Accel data match or not.
    // We don't send a packet if they do in Async mode.
    accelDataDidNotMatchFlag = (*(uint32_t *)&AccelData != *(uint32_t *)&PrevAccelData);

    canTransmitFlag = (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_SYNC) ||
		      (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_ASYNC && accelDataDidNotMatchFlag);

    if (canTransmitFlag)
    {
        Packet_Put(PACKET_CMD_SEND_ACCEL_XYZ, AccelData.axes.x, AccelData.axes.y, AccelData.axes.z);
        // Update PrevData to the new data
        PrevAccelData = AccelData;
    }

    LEDs_Toggle(LED_GREEN);

  }
}

static void RTCSecondsChangeThread(void *arg)
{
  OS_ERROR error;

  uint8_t hrs, mins, secs;

  for (;;)
  {
    (void)OS_SemaphoreWait(RTC_SecondsChangeSemaphore, 0);
    RTC_Get(&hrs, &mins, &secs);
    // Send to PC and toggle the LED as per Tower protocol
    if (Packet_Put(PACKET_CMD_SET_TIME, hrs, mins, secs))
    {
      LEDs_Toggle(LED_YELLOW);
    }
  }
}

static void FTMChannelOutputEventThread(void *arg)
{
  // For use of the channel number if duplicating the thread
  #define channelNb ((uint8_t *)arg)

  OS_ERROR error;

  for (;;)
  {
    (void)OS_SemaphoreWait(FTM_ChannelOutputEventSemaphores[0], 0);
    LEDs_Off(LED_BLUE);

    /*
    switch (*channelNb)
    {
      case 0:
	(void)OS_SemaphoreWait(FTM_ChannelOutputEventSemaphores[0], 0);
	LEDs_Off(LED_BLUE);
	break;
    }
    */
  }
}

/*
** ###################################################################
**
**	Callback Functions:
**	...for FTM, PIT, RTC, Accel
**
** ###################################################################
*/
static void FTMCallbackFunction(void *args)
{
  LEDs_Off(LED_BLUE);
}

static void PITCallbackFunction(void *args)
{
  Accel_ReadXYZ(&AccelData);
}

static void AccelDataReadyCallbackFunction(void *args)
{
  Accel_ReadXYZ(&AccelData);
}

static void AccelDataReadCompleteCallbackFunction(void *args)
{

  static bool canTransmitFlag = false;

  canTransmitFlag = (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_ASYNC && (*(uint32_t *)&AccelData != *(uint32_t *)&PrevAccelData)) ||
		    (*NvTowerProtocolMode == TOWER_PROTOCOL_MODE_SYNC);

  if (canTransmitFlag)
  {
      Packet_Put(PACKET_CMD_SEND_ACCEL_XYZ, AccelData.axes.x, AccelData.axes.y, AccelData.axes.z);
      PrevAccelData = AccelData;
  }

  LEDs_Toggle(LED_GREEN);

}

static void RTCCallbackFunction(void *args)
{
  uint8_t hrs, mins, secs;
  RTC_Get(&hrs, &mins, &secs);
  if (Packet_Put(PACKET_CMD_SET_TIME, hrs, mins, secs))
  {
    LEDs_Toggle(LED_YELLOW);
  }
}

/**************************************************************************************************************/
/*
//lint -save  -e970 Disable MISRA rule (6.3) checking.
int main(void)
//lint -restore Enable MISRA rule (6.3) checking.
{
  // Write your local variable definition here
  // Processor Expert internal initialization. DON'T REMOVE THIS CODE!!!
  PE_low_level_init();
  // End of Processor Expert internal initialization.

  __DI();
  TowerInit();
  TowerSetup();
  __EI();

  // Packet handler thread
  // Write your code here

  // Don't write any code pass this line, or it will be deleted during code generation.
  // RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!!
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component.
  #endif
  // End of RTOS startup code.
  // Processor Expert end of main routine. DON'T MODIFY THIS CODE!!!
  for(;;){}
  // Processor Expert end of main routine. DON'T WRITE CODE BELOW!!!
} // End of main routine. DO NOT MODIFY THIS TEXT!!!

*/


/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
