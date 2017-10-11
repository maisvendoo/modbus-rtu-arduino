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

#include "modbus-rtu.h"

/// Supported functions
const uint8_t SUPPORTED_FUNC[] = 
	{
		MB_FUNC_NONE,
		MB_FUNC_READ_COILS,
		MB_FUNC_READ_DISCRETE_INPUT,
		MB_FUNC_READ_HOLDING_REGISTERS,
		MB_FUNC_READ_INPUT_REGISTERS,
		MB_FUNC_WRITE_COIL,
		MB_FUNC_WRITE_HOLDING_REGISTER,
		MB_FUNC_WRITE_MULTIPLE_COILS,
		MB_FUNC_WRITE_MULTIPLE_REGISTERS
	};

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
ModbusDevice::ModbusDevice(uint8_t device_id, 
						   uint8_t uart_id, 						   
						   uint8_t rs485_tx_enable_pin)
{
	this->device_id = device_id;
	this->uart_id = uart_id;
	this->rs485_tx_enable_pin = rs485_tx_enable_pin;	

	ready_time = 0;

	for (uint8_t i = 0; i < MAX_BUFFER_SIZE; i++)
		au8Buffer[i] = 0;

	u8RegsBuffSize = 0;

	this->u16Timeout = 1000000;	
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
ModbusDevice::~ModbusDevice()
{

}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void ModbusDevice::begin(uint32_t baudrate)
{
	// Save required baudrate
	this->baudrate = baudrate;

	// Modbus timings calculation
	this->quiet_time35 = (unsigned long) (QUIET_TIME_MULTIPLE / baudrate);

	// Serial port configuration
	serialPortConfig(this->uart_id, baudrate);
}

//-----------------------------------------------------------------------------
//	Poll of Modbus
//-----------------------------------------------------------------------------
int8_t ModbusDevice::poll(uint16_t * regs, uint8_t u8size)
{
	u8RegsBuffSize = u8size;

	// Get bytes available  for read
	uint8_t u8BytesInBuff = serialPort->available();

	// Is buffer clean?
	if (u8BytesInBuff == 0)
		return 0;

	if (u8BytesInBuff != u8LastRecivedBytes)
	{
		u8LastRecivedBytes = u8BytesInBuff;
		ready_time = micros() + quiet_time35;
		return 0;
	}

	if (micros() < ready_time)
		return 0;
		
	u8LastRecivedBytes = 0;

	// Get recieve buffer
	int8_t i8BuffState = getRxBuffer();

	u8LastError = i8BuffState;

	if (i8BuffState < 7)
		return i8BuffState;

	// Check slave ID
	if (au8Buffer[ID] != device_id)
		return 0;

	// Validate message: check CRC, function code, data size
	uint8_t u8Exception = validateRequest();

	if (u8Exception > 0)
	{
		if (u8Exception != NO_REPLY)
		{
			// Send exception message
			buildException(u8Exception);
			digitalWrite(7, HIGH);
			sendTxBuffer();
		}

		u8LastError = u8Exception;

		return u8Exception;
	}

	timeout = micros() + (unsigned long) u16Timeout;

	u8LastError = 0;

	// Parse message
	switch (au8Buffer[FUNC_CODE])
	{
	case MB_FUNC_READ_COILS:

	case MB_FUNC_READ_DISCRETE_INPUT:

		return processFunc_1_2(regs, u8size);

		break;

	case MB_FUNC_READ_INPUT_REGISTERS:

	case MB_FUNC_READ_HOLDING_REGISTERS:
				
		return processFunc_3_4(regs, u8size);

		break;

	case MB_FUNC_WRITE_COIL:

		return processFunc_5(regs, u8size);

		break;

	case MB_FUNC_WRITE_HOLDING_REGISTER:

		return processFunc_6(regs, u8size);

		break;

	case MB_FUNC_WRITE_MULTIPLE_COILS:

		return processFunc_15(regs, u8size);

		break;

	case MB_FUNC_WRITE_MULTIPLE_REGISTERS:

		return processFunc_16(regs, u8size);

		break;

	default:

		break;
	}

	return i8BuffState;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void ModbusDevice::serialPortConfig(uint8_t uart_id, uint32_t baudrate)
{
	// Check active UART
	switch (uart_id)
	{
#if defined(UBRR1H)
	case SERIAL_1:

		serialPort = &Serial1;
		break;
#endif

#if defined(UBRR2H)
	case SERIAL_2:

		serialPort = &Serial2;
		break;
#endif

#if defined(UBRR3H)
	case SERIAL_3:

		serialPort = &Serial1;
		break;
#endif

	case SERIAL_0:
	default:

		serialPort = &Serial;
		break;
	}

	// Set port parametres
	serialPort->begin(baudrate, SERIAL_8N1);

	// Configure RS485 TX enable pin as output
	if (rs485_tx_enable_pin > TX_PIN)
	{
		pinMode(rs485_tx_enable_pin, OUTPUT);
		digitalWrite(rs485_tx_enable_pin, LOW);
	}

	// Clean receive buffer
	while (serialPort->read() >= 0);

	// Clean main variables
	u8BufferSize = 0;
	u16InCount = u16OutCount = u16ErrorCount = 0;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
int8_t ModbusDevice::getRxBuffer()
{
	bool bBuffOverflow = false;

	// Set RS485 converter to recieve
	digitalWrite(rs485_tx_enable_pin, LOW);

	// Read serial buffer
	u8BufferSize = 0;

	while (serialPort->available())
	{
		// Read bytes from buffer
		au8Buffer[u8BufferSize] = serialPort->read();
		u8BufferSize++;

		// Check buffer overflow
		if (u8BufferSize > MAX_BUFFER_SIZE)
			bBuffOverflow = true;
	}

	// Count incomming messages
	u16InCount++;

	// Check buffer overflow
	if (bBuffOverflow)
	{
		u16ErrorCount++;
		return ERROR_BUFF_OVERFLOW;
	}

	return u8BufferSize;
}
//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void ModbusDevice::sendTxBuffer()
{
	// Calculate CRC for message
	uint16_t u16crc = calcCRC(u8BufferSize);

	au8Buffer[u8BufferSize] = u16crc >> 8;
	u8BufferSize++;
	au8Buffer[u8BufferSize] = u16crc & 0x00FF;
	u8BufferSize++;	

	// Set transfer completion flag 
	switch(uart_id)
	{
#if defined(UBRR1H)
	case SERIAL_1:

		UCSR1A = UCSR1A | (1 << TXC1);

		break;
#endif

#if defined(UBRR2H)
	case SERIAL_2:

		UCSR2A = UCSR2A | (1 << TXC2);

		break;
#endif

#if defined(UBRR3H)
	case SERIAL_3:

		UCSR3A = UCSR3A | (1 << TXC3);

		break;
#endif

	case SERIAL_0:

		UCSR0A = UCSR0A | (1 << TXC0);

		break;
	}

	// Set RS485 converter to transmission mode
	digitalWrite(rs485_tx_enable_pin, HIGH);

	// Transmit buffer
	serialPort->write(au8Buffer, u8BufferSize);
	
	// Wait transfer time
	switch (uart_id)
	{
#if defined(UBRR1H)
	case SERIAL_1:

		while (!(UCSR1A & (1 << TXC1)));
		break;
#endif

#if defined(UBRR2H)
	case SERIAL_2:

		while (!(UCSR2A & (1 << TXC2)));
		break;
#endif

#if defined(UBRR3H)
	case SERIAL_3:

		while (!(UCSR3A & (1 << TXC3)));
		break;
#endif

	case SERIAL_0:

		while (!(UCSR0A & (1 << TXC0)));
		break;
	}
	
	// Set RS485 converter to recieve mode
	digitalWrite(rs485_tx_enable_pin, LOW);

	while (serialPort->read() >= 0);	

	u8BufferSize = 0;

	timeout = micros() + (unsigned long) u16Timeout;

	u16OutCount++;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
uint8_t ModbusDevice::validateRequest()
{
	// Get message CRC
	uint16_t u16MsgCRC = au8Buffer[u8BufferSize - 2] << 8;
	u16MsgCRC |= au8Buffer[u8BufferSize - 1];

	// Calc recived data CRC
	if (calcCRC(u8BufferSize - 2) != u16MsgCRC)
	{
		u16ErrorCount++;
		return NO_REPLY;
	}

	// Check function code
	bool isSupported = false;

	for (uint8_t i = 0; i < sizeof(SUPPORTED_FUNC); i++)
	{
		if (SUPPORTED_FUNC[i] == au8Buffer[FUNC_CODE])
		{
			isSupported = true;
			break;
		}
	}

	if (!isSupported)
	{
		u16ErrorCount++;
		return EXC_FUNC_CODE;
	}

	// Check data structs 
	uint16_t u16regs = 0;
	uint8_t u8regs = 0;

	switch (au8Buffer[FUNC_CODE])
	{
	case MB_FUNC_READ_COILS:

	case MB_FUNC_READ_DISCRETE_INPUT:

	case MB_FUNC_WRITE_MULTIPLE_COILS:

		// Check size of data structute
		u16regs = ((au8Buffer[ADDRESS_HI] << 8) | au8Buffer[ADDRESS_LO]) / 16;
		u16regs += ((au8Buffer[NB_HI] << 8) | au8Buffer[NB_LO]) / 16;
		u8regs = (uint8_t) u16regs;

		if (u8regs > u8RegsBuffSize)
			return EXC_ADDR_RANGE;

		break;

	case MB_FUNC_WRITE_COIL:

		u16regs = ((au8Buffer[ADDRESS_HI] << 8) | au8Buffer[ADDRESS_LO]) / 16;
		u8regs = (uint8_t) u16regs;

		if (u8regs > u8RegsBuffSize)
			return EXC_ADDR_RANGE;

		break;

	case MB_FUNC_WRITE_HOLDING_REGISTER:

		u16regs = ((au8Buffer[ADDRESS_HI] << 8) | au8Buffer[ADDRESS_LO]);
		u8regs = (uint8_t) u16regs;

		if (u8regs > u8RegsBuffSize)
			return EXC_ADDR_RANGE;
		
		break;

	case MB_FUNC_READ_HOLDING_REGISTERS:

	case MB_FUNC_READ_INPUT_REGISTERS:

	case MB_FUNC_WRITE_MULTIPLE_REGISTERS:

		u16regs = (au8Buffer[ADDRESS_HI] << 8) | au8Buffer[ADDRESS_LO];
		u16regs += (au8Buffer[NB_HI] << 8) | au8Buffer[NB_LO];
		u8regs = (uint8_t) u16regs;

		if (u8regs > u8RegsBuffSize)
			return EXC_ADDR_RANGE;

		break;
	}

	return 0;
}

//-----------------------------------------------------------------------------
//	CRC calculation
//-----------------------------------------------------------------------------
uint16_t ModbusDevice::calcCRC(uint8_t u8_buff_size)
{
#if defined(CRC_TABLE)
	return calcCRC_table(u8_buff_size);
#else
	return calcCRC_realtime(u8_buff_size);
#endif
}

//-----------------------------------------------------------------------------
//	Realtime CRC calculation
//-----------------------------------------------------------------------------
uint16_t ModbusDevice::calcCRC_realtime(uint8_t u8_buff_size)
{
	uint32_t tmp, tmp2, flag;
	tmp = 0xFFFF;

	for (uint8_t i = 0; i < u8_buff_size; i++)
	{
		tmp = tmp ^ au8Buffer[i];

		for (uint8_t j = 1; j <= 8; j++)
		{
			flag = tmp & 0x0001;
			tmp >>= 1;

			if (flag)
				tmp ^= 0xA001;
		}
	}

	tmp2 = tmp >> 8;
	tmp = (tmp << 8) | tmp2;
	tmp &= 0xFFFF;

	return tmp;
}

//-----------------------------------------------------------------------------
//	Calculate CRC by table algorithm
//-----------------------------------------------------------------------------
uint16_t ModbusDevice::calcCRC_table(uint8_t u8_buff_size)
{
	

	return 0;
}

//-----------------------------------------------------------------------------
//	Configure exception message
//-----------------------------------------------------------------------------
void ModbusDevice::buildException(uint8_t u8_exception)
{
	uint8_t u8func = au8Buffer[FUNC_CODE];

	au8Buffer[ID] = device_id;
	au8Buffer[FUNC_CODE] = u8func + 0x80;
	au8Buffer[2] = u8_exception;
	u8BufferSize = EXCEPTION_SIZE;
}

//-----------------------------------------------------------------------------
//	Read discrete inputs or coils
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_1_2(uint16_t * regs, uint8_t u8size)
{
	uint8_t u8CurReg = 0;
	uint8_t u8CurBit = 0;
	uint8_t u8BytesNum = 0;
	uint8_t u8BitsNum = 0;

	uint8_t u8CopyBuffSize = 0;
	uint16_t u16CurCoil = 0;
	uint16_t u16Coil = 0;

	uint16_t u16StartCoil = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);
	uint16_t u16CoilNum = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);

	u8BytesNum = (uint8_t) (u16CoilNum / 8);

	if (u16CoilNum % 8 != 0)
		u8BytesNum++;

	au8Buffer[ADDRESS_HI] = u8BytesNum;
	u8BufferSize = ADDRESS_LO;

	u8BitsNum = 0;

	for (u16CurCoil = 0; u16CurCoil < u16CoilNum; u16CurCoil++)
	{
		u16Coil = u16StartCoil + u16CurCoil;
		u8CurReg = (uint8_t)(u16Coil / 16);
		u8CurBit = (uint8_t)(u16Coil % 16);

		bitWrite(au8Buffer[u8BufferSize], u8BitsNum, bitRead(regs[u8CurReg], u8CurBit));

		u8BitsNum++;

		if (u8BitsNum > 7)
		{
			u8BitsNum = 0;
			u8BufferSize++;
		}
	}

	if (u16CoilNum % 8 != 0)
		u8BufferSize++;

	u8CopyBuffSize = u8BufferSize + 2;

	sendTxBuffer();

	return u8CopyBuffSize;
}

//-----------------------------------------------------------------------------
//	Read input or holding registers
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_3_4(uint16_t * regs, uint8_t size)
{
	// Get start address from master message
	uint8_t u8StartAddr = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);
	// Get registers number from master message
	uint8_t u8RegsNum = word(au8Buffer[NB_HI],  au8Buffer[NB_LO]);
	// Size of buffer (with CRC)
	uint8_t u8CopyBuffSize = 0;

	// Save registers number into buffer 
	au8Buffer[2] = u8RegsNum << 1;
	// Initial buffer size (ID + func code + registers number)
	u8BufferSize = 3;

	// Save registers data into buffer
	for (uint8_t i = u8StartAddr; i < u8StartAddr + u8RegsNum; i++)
	{
		au8Buffer[u8BufferSize] = highByte(regs[i]);
		u8BufferSize++;
		au8Buffer[u8BufferSize] = lowByte(regs[i]);
		u8BufferSize++;
	}

	u8CopyBuffSize = u8BufferSize + 2;

	// Transmit buffer to master
	sendTxBuffer();	

	return u8CopyBuffSize;
}

//-----------------------------------------------------------------------------
//	Write single coil
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_5(uint16_t * regs, uint8_t u8size)
{
	uint8_t u8CurReg = 0; // current register
	uint8_t u8CurBit = 0; // current bit
	uint8_t u8CopyBuffSize = 0; 

	// Get coil address from master's message
	uint16_t u16Coil = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);

	// Calculate number of register and number of bit in internal data array
	u8CurReg = (uint8_t) (u16Coil / 16);
	u8CurBit = (uint8_t) (u16Coil % 16);

	// Write required bit
	bitWrite(regs[u8CurReg], u8CurBit, au8Buffer[NB_HI] == 0xFF);

	// Send answer to master
	u8BufferSize = RESPOSE_SIZE;
	u8CopyBuffSize = u8BufferSize + 2;
	sendTxBuffer();

	return u8CopyBuffSize;
}

//-----------------------------------------------------------------------------
//	Write single register
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_6(uint16_t * regs, uint8_t u8size)
{
	// Get register address from master message
	uint8_t u8Address = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);
	uint8_t u8CopyBuffSize = 0;
	// Get register value from master message
	uint16_t u16Value = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);

	// Write value in internal data array
	regs[u8Address] = u16Value;

	// Send answer to master
	u8BufferSize = RESPOSE_SIZE;
	u8CopyBuffSize = u8BufferSize + 2;
	sendTxBuffer();

	return u8CopyBuffSize;
}

//-----------------------------------------------------------------------------
//	Write multiple coils
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_15(uint16_t * regs, uint8_t u8size)
{
	uint8_t u8CopyBuffSize = 0;

	uint8_t u8CurReg = 0;
	uint8_t u8CurBit = 0;
	uint8_t u8BitsNum = 0;
	uint8_t u8FrameByte = 0;

	uint16_t u16CurCoil = 0;
	uint16_t u16Coil = 0;

	bool bTmp = false;

	// Get first coil address
	uint16_t u16StartCoil = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);
	// Get coils number
	uint16_t u16CoilsNum = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);

	// Number of writet bits
	u8BitsNum = 0;
	// Begin of coils frame in data array
	u8FrameByte = 7;

	// Set each coil
	for (u16CurCoil = 0; u16CurCoil < u16CoilsNum; u16CurCoil++)
	{
		u16Coil = u16StartCoil + u16CurCoil;
		u8CurReg = (uint8_t) (u16Coil / 16);
		u8CurBit = (uint8_t) (u16Coil % 16);

		bTmp = bitRead(au8Buffer[u8FrameByte], u8BitsNum);

		bitWrite(regs[u8CurReg], u8CurBit, bTmp);

		u8BitsNum++;

		if (u8BitsNum > 7)
		{
			u8BitsNum = 0;
			u8FrameByte++;
		}
	}

	// Send answer to master
	u8BufferSize = 6;
	u8CopyBuffSize = u8BufferSize + 2;
	sendTxBuffer();

	return u8CopyBuffSize;
}

//-----------------------------------------------------------------------------
//	Write multiple registers
//-----------------------------------------------------------------------------
int8_t ModbusDevice::processFunc_16(uint16_t * regs, uint8_t u8size)
{
	uint8_t u8CopyBuffSize = 0;

	uint8_t u8StartAddr = word(au8Buffer[ADDRESS_HI], au8Buffer[ADDRESS_LO]);
	uint8_t u8RegsNum = word(au8Buffer[NB_HI], au8Buffer[NB_LO]);
	uint16_t tmp = 0;

	au8Buffer[NB_HI] = 0;
	au8Buffer[NB_LO] = u8RegsNum;
	u8BufferSize = RESPOSE_SIZE;

	for (uint16_t i = 0; i < u8RegsNum; i++)
	{
		tmp = word(au8Buffer[(BYTE_COUNT + 1) + i * 2], au8Buffer[(BYTE_COUNT + 2) + i * 2]);
		regs[u8StartAddr + i] = tmp;
	}

	u8CopyBuffSize = u8BufferSize + 2;
	sendTxBuffer();

	return u8CopyBuffSize;
}
