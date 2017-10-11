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

#ifndef MODBUS_H
#define MODBUS_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//class Modbus;

#define		UART_ID					0
#define		UART_BAUDRATE			115200

// Размер регистрового буфера
#define		BUFFER_SIZE		11

#define		HOLDING_REGS_INIT_ADDR	0x5
#define		COILS_INIT_ADDR			0x10

// Минимальное и максимальное смещение для регистров ввода
#define		INPT_MIN_OFFSET	2
#define		INPT_MAX_OFFSET 4

/*!
 * \class
 * \brief Шина ModBus
 */
#include "modbus-rtu.h"

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
class CModbus
{
protected:

	/// Объект, реализующий логику ведомого устройства
	ModbusDevice *slave;

	/// Скорость UART
	uint32_t baudrate;

	/// Пин управляющий режимом RS485
	uint8_t rs485_tx_enable;

	/// Регистры ModBus
	uint16_t au16data[BUFFER_SIZE] = { 0 };

public:

	CModbus() {};

	CModbus(uint8_t slave_id, 
		    uint8_t uart_id, 
		    uint8_t rs485_tx_enable, 
		    uint32_t baudrate);
	
	/// Инициализация
	void init();

	/// Логика работы
	void process();		

	/// Записать значение в InputRegister по адресу offset
	void setInputRegister(uint16_t offset, uint16_t value);

	/// Установить значение бита дискретного входа
	void setDiscreteInput(uint8_t bit, bool value);	

	/// Прочесть значение Holding Register по адресу offset
	uint16_t getHoldingRegister(uint16_t offset);

	/// Прочесть значение Coil по адресу offset
	bool getCoil(uint16_t offset);

	/// Установить значение Coil
	void setCoil(uint16_t offset, bool value);

	/// Установить значение Holding Register
	void setHoldingReg(uint16_t offset, uint16_t value);
};

extern CModbus ModbusClass;

#endif

