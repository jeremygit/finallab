/*
 *  @file Flash.c
 *
 *  @brief Implementation of a Flash Hardware Abstraction Layer for accessing Flash.
 *
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 *
 *  @date Created on: 10 Apr 2017
 *
 */

/*!
 *  @addtogroup Flash_module Flash module documentation
 *  @{
*/
/* MODULE Flash */

// Information regarding the memory map for MK70F12 processor.
#include "MK70F12.h"
// New non-standard types
#include "types.h"

#include "Flash.h"
//#include "packet.h"

// Half Word Address related assets
#define ADDRESS_HWORD_MASK 0x00000001
#define IS_HWORD_ALIGNED(address) ((((uint32_t)address) & ADDRESS_HWORD_MASK) == 0)
// Returns a Half Word aligned address from any given address.
#define HWORD_ALIGNED_ADDRESS(address) (volatile uint16_t *)(((uint32_t)address) & ~ADDRESS_HWORD_MASK)

// Word Address related assets.
#define ADDRESS_WORD_MASK  0x00000003
#define IS_WORD_ALIGNED(address) ((((uint32_t)address) & ADDRESS_WORD_MASK) == 0)
// Returns a Word aligned address from any given address.
#define WORD_ALIGNED_ADDRESS(address) (volatile uint32_t *)(((uint32_t)address) & ~ADDRESS_WORD_MASK)

// Phrase Address related assets
#define ADDRESS_PHRASE_MASK 0x0000007
#define IS_PHRASE_ALIGNED(address) ((((uint32_t)address) & ADDRESS_PHRASE_MASK) == 0)
// Returns a Phrase aligned address from any given address.
#define PHRASE_ALIGNED_ADDRESS(address) (volatile uint64_t *)(((uint32_t)address) & ~ADDRESS_PHRASE_MASK)

// FCCOB Related Commands
#define CMD_WRITE 0x07
#define CMD_ERASE 0x09

typedef enum
{
  BYTE_SIZE = 1,
  HWORD_SIZE = 2,
  WORD_SIZE = 4
} DATA_SIZES;

// A datatype to conveniently structure an FCCOB
typedef struct TCCOB
{
  uint8_t cmd;
  union
  {
    uint32_t bits : 24;
    struct
    {
      uint8_t bits7to0;
      uint8_t bits15to8;
      uint8_t bits23to16;
    };
  } address;
  union
  {
    uint64_t bits;
    struct
    {
      uint8_t bytes7; // Will arrange data given in little endian
      uint8_t bytes6; // To the big endian of Flash...we we go to use it.
      uint8_t bytes5;
      uint8_t bytes4;
      uint8_t bytesB;
      uint8_t bytesA;
      uint8_t bytes9;
      uint8_t bytes8;
    };
  } data;
} TCCOB; /*!< FCCOB Data Structure */

/*! @brief Carries out the Flash command procedure.
 *
 *  @param address is the start address of the Phrase.
 *
 *  @param phrase contains the phrase data to write to Flash.
 *
 *  @return bool - TRUE if the command was successful.
 */
static bool LaunchCommand(TCCOB* cmdObj);

/*! @brief Sets up a command to write to Flash then executes it.
 *
 *  @param address is the start address of the Phrase.
 *
 *  @param phrase contains the phrase data to write to Flash.
 *
 *  @return bool - TRUE if the command was successful.
 *  @note Assumes a valid Phrase address will be supplied.
 */
static bool WritePhrase(const uint32_t address, const uint64union_t phrase);

/*! @brief Carries out both the erasing and writing of new Phrase data as to modify a Phrase in Flash.
 *
 *  @param address is the start address of the Phrase.
 *
 *  @param phrase contains the already updated Phrase data we want to modify the old Phrase data with.
 *
 *  @return bool - TRUE if both erase and write were successful.
 *  @note Assumes a valid Phrase address will be supplied.
 */
static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase);

/*! @brief Sets up a command to perform the erasing of a Sector in Flasha and executes it.
 *
 *  @param address is the start address for the Sector.
 *
 *  @return bool - TRUE if the erasure operation was successful.
 *  @note Assumes a valid Sector address will be supplied.
 */
static bool EraseSector(const uint32_t address);

/*! @brief Checks a given address is within the Flash address space we are using.
 *
 *  @param address will be an address attempting to be programmed or read from that we want to check.
 *
 *  @return bool - TRUE if the address is in range.
 */
static bool AddressInRange(const uint32_t address);



bool Flash_Init(void)
{
  return true;
}

bool Flash_AllocateVar(volatile void** variable, const uint8_t datasize)
{

  // TODO check if full, to exit early

  /* A field of bits where each bit represents the position of a Byte in the phrase
  // Bits are assigned from <- left to right*/
  static uint8_t occupiedBytesField;

  // The size of Flash data we are using, used when allocated Flash addresses so we don't
  // allocate beyond the address space.
  static const uint8_t maxAllocatableSize = 1 + (FLASH_DATA_END - FLASH_DATA_START);

  // A count shifts masks and becomes the Flash address offset
  uint8_t bitCount = 0;

  /* Calculate a bit mask that represents the amount of bytes the variable would occupy
  // e.g.	1 * 1 - 1 | 1 = 0001
  //		2 * 2 - 1 | 1 = 0011
  //		4 * 4 - 1 | 1 = 1111*/
  uint8_t byteMask = ((datasize * datasize) - 1) | 1;

  while (bitCount < maxAllocatableSize)
  {
      // Shift to a position in the occupiedBytes field and check
      // whether the position is occupied and whether additional bytes will be able to fit
      if ( ((occupiedBytesField >> bitCount) & byteMask) == 0) 		// && (bitCount % datasize) == 0
      {
	  switch(datasize)
	  {
	    // Give the variables a Flash address at the offset
	    case BYTE_SIZE:
	      *variable = &(_FB(FLASH_DATA_START + bitCount));
	      break;
	    case HWORD_SIZE:
	      *variable = &(_FH(FLASH_DATA_START + bitCount));
	      break;
	    case WORD_SIZE:
	      *variable = &(_FW(FLASH_DATA_START + bitCount));
	      break;
	    default: return false;
	  }
	  // if the data can fit, store the bytes consumed in the occupiedBytesField
	  occupiedBytesField |= (byteMask << bitCount);
	  return true;
      }
      // if no space was found at the count location
      // jump to the next address offset the data should be aligned to
      bitCount += datasize;
  }
  // No position was found...
  return false;
}

bool Flash_Write32(volatile uint32_t* const address, const uint32_t data)
{
  if (!IS_WORD_ALIGNED(address))
  {
      return false;
  }

  // Get the closest Phrase address based on the given Word address
  volatile uint64_t *flashAddress = PHRASE_ALIGNED_ADDRESS(address);
  // Read a Phrase from Flash.
  uint64union_t flashData = ((uint64union_t)_FP(flashAddress));

  if (address > flashAddress)
  {
      flashData.s.Hi = data; // address was in the higher portion of the phrase
  } else {
      flashData.s.Lo = data;
  }
  return ModifyPhrase((uint32_t)flashAddress, flashData);
}

bool Flash_Write16(volatile uint16_t* const address, const uint16_t data)
{
  if (!IS_HWORD_ALIGNED(address))
  {
      return false;
  }

  // Get the closest Word address based on the given Half-Word address
  volatile uint32_t *flashAddress = WORD_ALIGNED_ADDRESS(address);
  // Read a Word from Flash.
  uint32union_t flashData = ((uint32union_t)_FW(flashAddress));

  if (address > flashAddress)
  {
      flashData.s.Hi = data; // address was in the higher portion of the Word
  } else {
      flashData.s.Lo = data;
  }
  return Flash_Write32(flashAddress, flashData.l);
}

bool Flash_Write8(volatile uint8_t* const address, const uint8_t data)
{
  if (!AddressInRange(address))
  {
      return false;
  }

  // Get the closest Half-Word address based on the given Byte address
  volatile uint16_t *flashAddress = HWORD_ALIGNED_ADDRESS(address);
  // Read a Half-Word from Flash.
  uint16union_t flashData = ((uint16union_t)_FH(flashAddress));

  if (address > flashAddress)
  {
      flashData.s.Hi = data; // address was in the higher portion of the Half-Word
  } else {
      flashData.s.Lo = data;
  }
  return Flash_Write16(flashAddress, flashData.l);
}

bool Flash_Erase(void)
{
  EraseSector(FLASH_DATA_START);
  return true;
}

bool Flash_ReadByte(volatile uint8_t* const address, uint8_t *dataPtr)
{
  if (!AddressInRange(address))
  {
    return false;
  }
  *dataPtr = _FB(address);
  return true;
}

static bool LaunchCommand(TCCOB* cmdObj)
{
  //check for previous errors and clear them.
  if (FTFE_FSTAT & FTFE_FSTAT_FPVIOL_MASK ||
      FTFE_FSTAT & FTFE_FSTAT_ACCERR_MASK)
  {
      FTFE_FSTAT = FTFE_FSTAT_ACCERR_MASK;
      FTFE_FSTAT = FTFE_FSTAT_FPVIOL_MASK;
  }

  //command
  FTFE_FCCOB0 = cmdObj->cmd;
  //address
  FTFE_FCCOB1 = cmdObj->address.bits23to16;
  FTFE_FCCOB2 = cmdObj->address.bits15to8;
  FTFE_FCCOB3 = cmdObj->address.bits7to0;
  //LS-W
  FTFE_FCCOB4 = cmdObj->data.bytes4;
  FTFE_FCCOB5 = cmdObj->data.bytes5;
  FTFE_FCCOB6 = cmdObj->data.bytes6;
  FTFE_FCCOB7 = cmdObj->data.bytes7;
  // MS-W
  FTFE_FCCOB8 = cmdObj->data.bytes8;
  FTFE_FCCOB9 = cmdObj->data.bytes9;
  FTFE_FCCOBA = cmdObj->data.bytesA;
  FTFE_FCCOBB = cmdObj->data.bytesB;

  // Launch Command FTFE_FSTAT |= FTFE_FSTAT_CCIF_MASK;
  FTFE_FSTAT = FTFE_FSTAT_CCIF_MASK;

  // Wait until command has completed executing...
  while((FTFE_FSTAT & FTFE_FSTAT_CCIF_MASK)==0) {};

  return true;
}

static bool ModifyPhrase(const uint32_t address, const uint64union_t phrase)
{
  return EraseSector(address)
      && WritePhrase(address, phrase);
}

static bool WritePhrase(const uint32_t address, const uint64union_t phrase)
{
  TCCOB command = {
      CMD_WRITE,
      address,
      phrase.l
  };
  return LaunchCommand(&command);
}

static bool EraseSector(const uint32_t address)
{
  TCCOB command = {
      CMD_ERASE,
      address,
      0
  };
  return LaunchCommand(&command);
}

static bool AddressInRange(const uint32_t address)
{
  return address >= FLASH_DATA_START && address <= FLASH_DATA_END;
}

/*!
** @}
*/

/*
#define OCCUPIED_BYTES 0x00 //Global, acting as a register to record the position of the bytes occupied in flash memory
#define BYTE 1
#define HALF_WORD 2
#define WORD 4
#define STORAGE_SIZE 8
*/

/*
bool Flash_AllocateVarOld(volatile void** variable, const uint8_t datasize)
{

      uint8_t bit = 0,freebytes = 0; //position of the 8 bit number and number of free space available

      if (OCCUPIED_BYTES == 0xFF)
	  return False;
      while(bit < STORAGE_SIZE)
	{
	    //If bits are empty,increase counter of free space variable.
	    if(Bit_Check(OCCUPIED_BYTES,bit++) == 0)
	    {
	      freebytes++;
	    }
	    else
	    {
	      freebytes = 0; //no empty location found
	      continue;
	    }

	  //Test data correspondingly to their data size; left side of expression test for if are even,
	  //right expression to determine if there is enough space to allocate for data
	  if((freebytes % datasize == 0) && (freebyte == datasize))
	  {
	    if(AddrAllocate(variable,bit-1,datasize))
	      break;
	    else
	      return False;
	  }
      }
      return true;
}
*/

/*
bool Bit_Check(uint8_t x,uint8_t y) //To check every bit of the 8 bytes
{
  if ((x >> (y)) & 1) // x = occupied bit, y = bit to be tested if full
    return true;
  else
    return false;
}
*/

/*
static AddrAllocate(volatile void**data,const uint8_t position, uint8_t datasize)
{
  switch(datasize)
  {
    case BYTE: //Assign appropriate address and indicate one byte has been allocated in Flash
      *(volatile uint8_t**) data = &(_FB(FLASH_DATA_START+position));
      // Values is written in binary form for easier understanding of how the flags are set for different data types.
      OCCUPIED_BYTES |= (00000001 << position);
      break;

    case HALF_WORD: //Assign appropriate address and indicate a half-word has been allocated in Flash
      *(volatile uint8_t**) data = &(_FB(FLASH_DATA_START+(position-1)));
      OCCUPIED_BYTES |= (00000011 << (position-1));
      break;

    case WORD: //Assign appropriate address and indicate a word has been allocated in Flash
      *(volatile uint8_t**) data = &(_FB(FLASH_DATA_START+(position-3)));
      OCCUPIED_BYTES |= (00001111 << (position-3));
      break;

    default : return False;
  }
  return True;
}
*/

