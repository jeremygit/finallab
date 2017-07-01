/*! @file accel.c
 *
 *  @brief Accelerometer HAL that communicates via I2C
 *
 *  @author Author: Jeremy Heritage 12033445 , Victor WU (12009155)
 *  @date Created on: 7 Jun 2017
 */

/*!
 *  @addtogroup <Accelerometer>
 *  @{
*/

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// CPU and PE_types are needed for critical section variables and the defintion of NULL pointer
#include "CPU.h"
#include "PE_types.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01

#define ADDRESS_INT_SOURCE 0x0C

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t               : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     		INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY	INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT	CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE	CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT	CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS	CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO	CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP	CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		    CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	    CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT	CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE	CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT	CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS	CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4            		CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	  CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT	CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE	CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT	CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS	CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	  CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	  CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

/**************************************************************************/
// Accelerometer configuration settings.
#define ACCEL_DEVICE_ADDRESS 0x1D //SA0 = 1
#define ACCEL_BAUD 100000
#define ACCEL_DATA_WINDOW_SIZE 3

// 0	0000 Interrupts disabled
// 8	1000 On Logic 0
// 9	1001 Rising Edge
// A	1010 Falling Edge
// B	1011 Either Rise or Fall
// C	1100 On Logic 1

// For toggling the PORTB (Accel Data Ready) interrupts on and off.
static const uint8_t PORTB_INTERRUPTS_OFF = 0x0;
static const uint8_t PORTB_INTERRUPTS_ON = 0xA;

// A Fag which blocks calls to the Accelerometer read function
// if there is already one in progress...
static bool ReadInProgress;

// To store the current setting of the Accelerometer
static volatile TAccelMode AccelReadMode;

// The Accelerometer reads X, Y and Z data, there for we always want to
// tell the I2C to read 3 bytes
static const uint8_t ACCEL_XYZ_DATA_SIZE = 3;

// Stores a history of 3 acceleromter XYZ readings
static TAccelData AccelDataWindow[ACCEL_DATA_WINDOW_SIZE];

// The user provided data pointer we want to read Accel data into
static TAccelData * UserXYZDataOutputPtr;

/** Called after completion an I2C read operation initiated by the accel module */
static void AccelReadI2CComplete(void);
static void *I2CReadCompleteArguments;

/** User defined callback for when Accel asserts a data-ready interrupt */
static void (*UserAccelDataReadyFunction)(void*);
static void *UserAccelDataReadyArguments;

/**
 * User defined callback when either IntRead or Pollreads complete
 * For example, the user may Packet_Put the new Accel data in the main
 * */
static void (*UserAccelDataReadCompleteFunction)(void*);
static void *UserAccelDataReadCompleteArguments;

/** Handle the operation of configuring the acceleromter read mode based on the mode-setting. */
static void ConfigureAccelForReadMode(void);

/** Perform the processing of the acceleromter data such as removing a previous data entry and applyign a median filter */
static void ProcessNewAccelData(void);

/** A RTOS Mutex so that no more that one call to the Accel read can happen, thus protecting the global data variable */
static OS_ECB *AccelNotInUseSemaphore;

OS_ECB *Accel_EventSemaphore;
OS_ECB *Accel_ReadCompleteSemaphore;


/**************************************************************************/

bool Accel_Init(const TAccelSetup* const accelSetup)
{

  /**************************************************************************/
  // Init global module params

  Accel_EventSemaphore = OS_SemaphoreCreate(0);
  Accel_ReadCompleteSemaphore = OS_SemaphoreCreate(0);

  AccelNotInUseSemaphore = OS_SemaphoreCreate(1);

  // Initialize user defined functions and arguments
  UserAccelDataReadCompleteFunction = accelSetup->readCompleteCallbackFunction;
  UserAccelDataReadCompleteArguments = accelSetup->readCompleteCallbackArguments;

  UserAccelDataReadyFunction = accelSetup->dataReadyCallbackFunction;
  UserAccelDataReadyArguments = accelSetup->dataReadyCallbackArguments;

  // Default
  AccelReadMode = ACCEL_POLL;

  // Initialize
  I2CReadCompleteArguments = NULL;

  ReadInProgress = false;

  /**************************************************************************/
  // I2C Setup

  TI2CModule i2cSetup = {
      ACCEL_DEVICE_ADDRESS,
      ACCEL_BAUD,
      AccelReadI2CComplete,
      I2CReadCompleteArguments
  };

  I2C_Init(&i2cSetup, accelSetup->moduleClk);

  // Configure Accelerometer
  // Put into standby mode
  I2C_Write(ADDRESS_CTRL_REG1, 0x00);

  // CTRL_REG3, potentially not setting
  // Active Low = 0 ... PORTB needs to be falling edge
  // Active Low = 1 ... PORTB needs to be rising
  CTRL_REG3_IPOL = 0;
  I2C_Write(ADDRESS_CTRL_REG3, CTRL_REG3);

  // Enable INT1 interrupts on the device
  CTRL_REG4_INT_EN_DRDY = 1;

  // Enable INT1 interrupts on the device
  CTRL_REG5_INT_CFG_DRDY = 1;

  // 0x3F -> Bit7=0 Bit6=0 Bit5-3(ODR 1.56hz)=111 reducedNoise=1 F_READ=1 activeMode=1
  // 0x3D -> Bit7=0 Bit6=0 Bit5-3(ODR 1.56hz)=111 reducedNoise=1 F_READ=0 activeMode=1
  CTRL_REG1_ACTIVE = 1;
  CTRL_REG1_F_READ = 1;
  CTRL_REG1_LNOISE = 1;
  CTRL_REG1_DR = DATE_RATE_1_56_HZ;
  CTRL_REG1_ASLP_RATE = SLEEP_MODE_RATE_1_56_HZ; // Necessary?
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);



  /**************************************************************************/
  // Init the PORTB for Accel interrupts

  // Enable PORTB
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

  // Set PTB4 to GPIO
  PORTB_PCR4 = PORT_PCR_MUX(1);

  // Clear an current interrupts (count be redundant from previous line)
  PORTB_PCR4 |= PORT_PCR_ISF_MASK;

  // Set the direction of the GPIO Pin 4
  GPIOB_PDDR &= ~0x08;

  // Enable interrupts if the AccelReadMode requires
  ConfigureAccelForReadMode();

  // Clear pending interrupts
  NVICICPR2 = (1 << 24);
  // Enable Interrupts
  NVICISER2 = (1 << 24);


}

static void ConfigureAccelForReadMode(void)
{
  // Disable accelerometer to adjust configuration
  I2C_Write(ADDRESS_CTRL_REG1, 0x00);

  if (AccelReadMode == ACCEL_POLL)
  {
    // Clear an interrupt config and then set, perhaps setting is not necessary
    PORTB_PCR4 &= ~PORT_PCR_IRQC_MASK;
    PORTB_PCR4 |= PORT_PCR_IRQC(PORTB_INTERRUPTS_OFF);

    // Disable all interrupts
    I2C_Write(ADDRESS_CTRL_REG4, 0x00);
    I2C_Write(ADDRESS_CTRL_REG5, 0x00);
  }
  else if (AccelReadMode == ACCEL_INT)
  {
    // Clear an interrupt config and then set
    PORTB_PCR4 &= ~PORT_PCR_IRQC_MASK;
    PORTB_PCR4 |= PORT_PCR_IRQC(PORTB_INTERRUPTS_ON);

    // Configure for interrupts as set in Init
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);
    I2C_Write(ADDRESS_CTRL_REG5, CTRL_REG5);
  }

  // Renable with same settings as in Init
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);

}

void Accel_ReadXYZ(uint8_t data[3])
{
  /*
  if (ReadInProgress)
  {
    return;
  }
  ReadInProgress = true;
  */

  (void)OS_SemaphoreWait(AccelNotInUseSemaphore, 0);

  UserXYZDataOutputPtr = (void *)data;

  if (AccelReadMode == ACCEL_POLL)
  {
    I2C_PollRead(ADDRESS_OUT_X_MSB, (void *)UserXYZDataOutputPtr, ACCEL_XYZ_DATA_SIZE);
    AccelReadI2CComplete();
  }
  else if (AccelReadMode == ACCEL_INT)
  {
    I2C_IntRead(ADDRESS_OUT_X_MSB, (void *)UserXYZDataOutputPtr, ACCEL_XYZ_DATA_SIZE);
    // AccelReadI2CComplete will come via I2C ISR
  }
}

void Accel_HandleI2CThread(void *arg)
{
  OS_ERROR error;
  for (;;)
  {
    (void)OS_SemaphoreWait(I2C_ReadCompleteSemaphore, 0);

    ProcessNewAccelData();
    //ReadInProgress = false;

    (void)OS_SemaphoreSignal(AccelNotInUseSemaphore);
    (void)OS_SemaphoreSignal(Accel_ReadCompleteSemaphore);
  }
}

// For polling
static void AccelReadI2CComplete(void)
{
  ProcessNewAccelData();
  //ReadInProgress = false;
  (void)OS_SemaphoreSignal(AccelNotInUseSemaphore);
  (void)OS_SemaphoreSignal(Accel_ReadCompleteSemaphore);
}

/* Read complete up 'til lab4
static void AccelReadI2CComplete(void)
{
  ProcessNewAccelData();

  if (UserAccelDataReadCompleteFunction)
  {
    (*UserAccelDataReadCompleteFunction)(UserAccelDataReadCompleteArguments);
  }

  ReadInProgress = false;
}
*/

void Accel_SetMode(const TAccelMode mode)
{
  OS_DisableInterrupts();
  //switch the mode of the accel
  AccelReadMode = mode;
  // then adjust configuration depending
  ConfigureAccelForReadMode();
  OS_EnableInterrupts();
}

static void ProcessNewAccelData(void)
{
  // Slide the window
  AccelDataWindow[0] = AccelDataWindow[1];
  AccelDataWindow[1] = AccelDataWindow[2];
  AccelDataWindow[2] = *UserXYZDataOutputPtr;

  UserXYZDataOutputPtr->axes.x = Median_Filter3(AccelDataWindow[0].axes.x,
						AccelDataWindow[1].axes.x,
						AccelDataWindow[2].axes.x);

  UserXYZDataOutputPtr->axes.y = Median_Filter3(AccelDataWindow[0].axes.y,
						AccelDataWindow[1].axes.y,
						AccelDataWindow[2].axes.y);

  UserXYZDataOutputPtr->axes.z = Median_Filter3(AccelDataWindow[0].axes.z,
						AccelDataWindow[1].axes.z,
						AccelDataWindow[2].axes.z);

}

void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  OS_ISREnter();

  // Check if interrupt was PTB4
  if (PORTB_PCR4 & PORT_PCR_ISF_MASK)
  {
    // Clear the interrupt
    PORTB_PCR4 |= PORT_PCR_ISF_MASK;

    (void)OS_SemaphoreSignal(Accel_EventSemaphore);
  }

  OS_ISRExit();
}

/** Interrupt up til Lab4 */
/*
void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  // Check if interrupt was PTB4
  if (PORTB_PCR4 & PORT_PCR_ISF_MASK)
  {
    // Clear the interrupt
    PORTB_PCR4 |= PORT_PCR_ISF_MASK;

    // TODO Read the accelerometer flag to check of data ready interrupt

    if (UserAccelDataReadyFunction)
    {
      (*UserAccelDataReadyFunction)(UserAccelDataReadyArguments);
    }
  }
}
*/

/*!
 * @}
*/
