//-----------------------------------------------------------------------------
//
//		Modbus RTU library for Arduino
//		(c) maisvendoo, 03/10/2017
//
//-----------------------------------------------------------------------------
/*
 * \file
 * \brief Modbus RTU library for Arduino
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 03/10/2017
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>

#include "modbus-types.h"

/*
 * \class
 * \brief Modbus device
 */
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
class ModbusDevice
{
public:

	/// Default constructor
	ModbusDevice() {}
	
	/// Universal constructor
	ModbusDevice(uint8_t device_id,
				 uint8_t uart_id,
				 uint8_t rs485_tx_enable_pin);

	/// Constructor for device with single hardware UART (uart_id = 0)
	ModbusDevice(uint8_t device_id, uint8_t rs485_tx_enable_pin)
	{
		ModbusDevice(device_id, 0, rs485_tx_enable_pin);
	}

	/// Destructor
	~ModbusDevice();

	/// Init Modbus device
	void begin(uint32_t baudrate);

	/// Poll Modbus
	int8_t poll(uint16_t *regs, uint8_t u8size);

private:

	HardwareSerial	*serialPort;			///< UART control object

	uint8_t			device_id;				///< Device ID (0 -- master; 1...247 -- slave)
	uint8_t			uart_id;				///< UART number
	uint8_t			rs485_tx_enable_pin;	///< RS485 converter TX control pin (0 -- receive; 1 -- transmit)
	uint32_t		baudrate;				///< UART baudrate

	unsigned long	quiet_time35;			///< Quiet time (3.5 symbol transfer time, us) 

	unsigned long	ready_time;				///< Time label of device ready for transmit or recieve

	unsigned long	timeout;

	uint8_t		u8BufferSize;					///< Receive buffer size
	uint8_t		u8LastRecivedBytes;				///< Number of last recived bytes
	uint8_t		u8LastError;					///< Last error code
	uint16_t	u16InCount;						///< Input messages count
	uint16_t	u16OutCount;					///< Output messages count
	uint16_t	u16ErrorCount;					///< Errors count

	uint16_t	u16Timeout;

	uint8_t		u8RegsBuffSize;

	uint8_t		au8Buffer[MAX_BUFFER_SIZE];	///< Recive buffer

	/// Serail port configuration
	void serialPortConfig(uint8_t uart_id, uint32_t baudrate);

	/// Get recieve buffer
	int8_t getRxBuffer();

	/// Send transmit buffer
	void sendTxBuffer();

	/// Validate request
	uint8_t validateRequest();

	/// Calculate CRC
	uint16_t calcCRC(uint8_t u8_buff_size);

	/// Realtime CRC calculation
	uint16_t calcCRC_realtime(uint8_t u8_buff_size);

	/// Table CRC calculation
	uint16_t calcCRC_table(uint8_t u8_buff_size);

	/// Build exception message
	void buildException(uint8_t u8_exception);

	/// Process functions 0x01 and 0x02
	int8_t processFunc_1_2(uint16_t *regs, uint8_t u8size);

	/// Process functions 0x03 and 0x04
	int8_t processFunc_3_4(uint16_t *regs, uint8_t u8size);	

	/// Process function 0x05
	int8_t processFunc_5(uint16_t *regs, uint8_t u8size);

	/// Process function 0x06
	int8_t processFunc_6(uint16_t *regs, uint8_t u8size);

	/// Process function 0x0F
	int8_t processFunc_15(uint16_t *regs, uint8_t u8size);

	/// Process function 0x10
	int8_t processFunc_16(uint16_t *regs, uint8_t u8size);
};

extern ModbusDevice CModbusRTU;

#endif // MODBUS_RTU_H

