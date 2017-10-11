//-----------------------------------------------------------------------------
//
//		
//
//
//
//-----------------------------------------------------------------------------
#ifndef MODBUS_TYPES_H
#define MODBUS_TYPES_H

/*
 * \enum
 * \brief Modbus functions
 */
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum MB_FUNC
{
	MB_FUNC_NONE = 0x00,
	MB_FUNC_READ_COILS = 0x01,
	MB_FUNC_READ_DISCRETE_INPUT = 0x02,
	MB_FUNC_READ_HOLDING_REGISTERS = 0x03,
	MB_FUNC_READ_INPUT_REGISTERS = 0x04,
	MB_FUNC_WRITE_COIL = 0x05,
	MB_FUNC_WRITE_HOLDING_REGISTER = 0x06,
	MB_FUNC_WRITE_MULTIPLE_COILS = 0x0F,
	MB_FUNC_WRITE_MULTIPLE_REGISTERS = 0x10
};

/*
* \enum
* \brief Multiplies for timing calculation
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum MB_TIMINGS_MULTIPLIES
{
	QUIET_TIME_MULTIPLE = 28000000	///< 8 bits * 1000000 us * 3.5
};

/*
* \enum
* \brief Hardware serial ports ID
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum SERIAL_PORT
{
	SERIAL_0 = 0x00,
	SERIAL_1 = 0x01,
	SERIAL_2 = 0x02,
	SERIAL_3 = 0x03
};

/*
* \enum
* \brief Pins reserved in Arduino
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum RESERVED_PINS
{
	RX_PIN = 0, ///< Rx pin
	TX_PIN = 1	///< Tx pin
};

/*
* \enum
* \brief Maximal data buffer size
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum 
{
	MAX_BUFFER_SIZE = 64
};

/*
* \enum
* \brief Modbus errors
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum MB_ERROR
{
	ERROR_NOT_MASTER	= -1,
	ERROR_POLLING		= -2,
	ERROR_BUFF_OVERFLOW	= -3,
	ERROR_BAD_CRC		= -4,
	ERR_EXCEPTION		= -5
};

/*
* \enum
* \brief Massage parts position in data buffer
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum MB_MESSAGE_POS
{
	ID = 0,
	FUNC_CODE,
	ADDRESS_HI,
	ADDRESS_LO,
	NB_HI,
	NB_LO,
	BYTE_COUNT
};

/*
* \enum
* \brief Exceptions codes
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum
{
	NO_REPLY = 255,
	EXC_FUNC_CODE = 1,
	EXC_ADDR_RANGE = 2,
	EXC_REGS_QUANT = 3,
	EXC_EXECUTE = 4
};

/*
* \enum
* \brief Sizes of some messages data
*/
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
enum
{
	RESPOSE_SIZE = 6,
	EXCEPTION_SIZE = 3,
	CHECKSUMM_SIZE = 2
};

#endif
