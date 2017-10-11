//-----------------------------------------------------------------------------
//
//		Реализация протокола ModBus на физ. уровне RS485
//		(c) РГУПС, ВЖД 14/06/2017
//		Разработал: Притыкин Д.Е.
//
//-----------------------------------------------------------------------------
/*!
* \file
* \brief Реализация протокола ModBus на физ. уровне RS485
* \copyright РГУПС, ВЖД
* \author Притыкин Д.Е.
* \date 14/06/2017
*/

//#include	<SoftwareSerial.h>
//#include	"ModbusRtu.h"

#include "modbus.h"

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
CModbus::CModbus(uint8_t slave_id, 
	             uint8_t uart_id, 
				 uint8_t rs485_tx_enable, 
				 uint32_t baudrate)
{
	// Создаем слейв с нужным ID
	slave = new ModbusDevice(slave_id, uart_id, rs485_tx_enable);

	// Сохраняем аппаратные настройки
	this->baudrate = baudrate;
	this->rs485_tx_enable = rs485_tx_enable;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::init()
{
	// Инициализируем пин управления приемом/передачей RS485
	pinMode(rs485_tx_enable, OUTPUT);
	digitalWrite(rs485_tx_enable, LOW);

	// ЗАпускаем слейв на заданной скорости обмена
	slave->begin(baudrate);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::process()
{
	int8_t state = slave->poll(au16data, BUFFER_SIZE);		
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::setInputRegister(uint16_t offset, uint16_t value)
{
	if ((offset < INPT_MIN_OFFSET) || (offset > INPT_MAX_OFFSET))
		return;

	au16data[offset] = value;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::setDiscreteInput(uint8_t bit, bool value)
{
	// Формируем маску
	uint16_t mask = 1;
	mask = mask << bit;

	if (value)
		au16data[0] = au16data[0] | mask;
	else
		au16data[0] = au16data[0] & (~mask);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
uint16_t CModbus::getHoldingRegister(uint16_t offset)
{
	if (offset > BUFFER_SIZE - 1)
		return 0;
	
	return au16data[offset];
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
bool CModbus::getCoil(uint16_t offset)
{
	// Вычисляем номер устанавливаемого бита
	int bit_number = offset - COILS_INIT_ADDR;

	if (bit_number < 0)
		return false;

	bool value = (bool) bitRead(au16data[1], bit_number);

	return value;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::setCoil(uint16_t offset, bool value)
{
	int bit_number = offset - COILS_INIT_ADDR;

	if (bit_number < 0)
		return;

	bitWrite(au16data[1], bit_number, value);
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void CModbus::setHoldingReg(uint16_t offset, uint16_t value)
{
	if (offset > BUFFER_SIZE - 1)
		return;

	au16data[offset] = value;
}

CModbus ModbusClass;

