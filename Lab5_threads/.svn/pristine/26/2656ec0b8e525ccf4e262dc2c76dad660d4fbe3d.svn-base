/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

/*!
**  @@defgroup Packet_module Packet module documentation
**  @{
*/
/* MODULE Packet */

#ifndef PACKET_H
#define PACKET_H

// new types
#include "types.h"

// Packet structure
#define PACKET_NB_BYTES 5

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[PACKET_NB_BYTES];     /*!< The packet as an array of bytes. */
  struct
  {
    uint8_t command;		      /*!< The packet's command. */
    union
    {
      struct
      {
        uint8_t parameter1;	      /*!< The packet's 1st parameter. */
        uint8_t parameter2;	      /*!< The packet's 2nd parameter. */
        uint8_t parameter3;	      /*!< The packet's 3rd parameter. */
      } separate;
      struct
      {
        uint16_t parameter12;         /*!< Parameter 1 and 2 concatenated. */
        uint8_t parameter3;
      } combined12;
      struct
      {
        uint8_t paramater1;
        uint16_t parameter23;         /*!< Parameter 2 and 3 concatenated. */
      } combined23;
    } parameters;
    uint8_t checksum;
  } packetStruct;
} TPacket;

#pragma pack(pop)


// Human readable bit masks for protocol commands
#define PACKET_CMD_STARTUP_VALUES 0x04		/*!< Packet Protocol Startup Values Command value */
#define PACKET_CMD_PROGRAM_FLASH_BYTE 0x07  	/*!< Packet Protocol Program Byte Command value */
#define PACKET_CMD_READ_FLASH_BYTE 0x08  	/*!< Packet Protocol Read Byte Command value */
#define PACKET_CMD_TOWER_VERSION 0x09		/*!< Packet Protocol Tower Version Command value */
#define PACKET_CMD_TOWER_NUMBER 0x0B		/*!< Packet Protocol Tower Number Command value */
#define PACKET_CMD_TOWER_MODE 0x0D  		/*!< Packet Protocol Tower Mode Command value */
#define PACKET_CMD_SET_TIME 0x0C		/*!< Packet Protocol Set Time Command value */

#define PACKET_CMD_SEND_ACCEL_XYZ 0x10		/*!< Packet Protocol Command Send Accelerometer XYZ values to PC */
#define PACKET_CMD_PROTOCOL_MODE 0x0A		/*!< Packet Protocol Command to set Tower Protocol */

#define PACKET_MODE_GET 0x01			/*!< Packet Protocol Command Get Mode value */
#define PACKET_MODE_SET 0x02			/*!< Packet Protocol Command Set Mode value */

#define Packet_Command     Packet.packetStruct.command
#define Packet_Parameter1  Packet.packetStruct.parameters.separate.parameter1
#define Packet_Parameter2  Packet.packetStruct.parameters.separate.parameter2
#define Packet_Parameter3  Packet.packetStruct.parameters.separate.parameter3
#define Packet_Parameter12 Packet.packetStruct.parameters.combined12.parameter12
#define Packet_Parameter23 Packet.packetStruct.parameters.combined23.parameter23
#define Packet_Checksum    Packet.packetStruct.checksum

extern TPacket Packet;

// Acknowledgement bit mask
extern const uint8_t PACKET_ACK_MASK;



/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk);

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

#endif

/*!
** @}
*/

