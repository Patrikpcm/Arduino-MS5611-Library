/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.
(c) 2016 Patrik Luiz Gogola - Campo Magro - Paraná - Brasil

Version: 1.0.0 29/07/2016

USES FLOATING-POINT EQUATIONS.

Forked from MS5611 library by Korneliusz Jarzebski and SFE_BMP180 library by Mike Grusin from SparkFun Electronics.

Like Mike Grusin from SparkFun says, this code uses the "beerware" license. You can redistribute it and/or modify
it under the term of pay me a beer someday, case you find it useful.
*/

#ifndef MS5611_h
#define MS5611_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MS5611
{
	public:
		MS5611(); // base type

		char begin();
			// call MS5611.reset() to initialize MS5611 before use
			// returns 1 if success, 0 if failure (bad component or I2C bus shorted?)
		
		char reset();
			// Send the RESET sequence to sensor to load factory calibration data
			// returns 1 if success, 0 if failure (bad component or I2C bus shorted?)

		char startTemperature(char oversampling);
			// command MS5611 to start a temperature measurement
			// oversampling: 0 - 4 for oversampling value
			// returns (number of ms to wait) for success, 0 for fail

		char getTemperature(double &T, byte compensate);
			// return temperature measurement from previous startTemperature command
			// places returned value in T variable (deg C)
			// returns 1 for success, 0 for fail
			// compensate: 1 or 0 for compensate temperature and pressure with a second order math. This increase accuracy for low temperature

		char startPressure(char oversampling);
			// command MS5611 to start a pressure measurement
			// oversampling: 0 - 4 for oversampling value
			// returns (number of ms to wait) for success, 0 for fail

		char getPressure(double &P, double &T, byte compensate);
			// return absolute pressure measurement from previous startPressure command
			// note: requires previous temperature measurement in variable T
			// places returned value in P variable (mbar)
			// returns 1 for success, 0 for fail
			// compensate: 1 or 0 for compensate temperature and pressure with a second order math. This increase accuracy for low temperature

		double sealevel(double P, double A);
			// convert absolute pressure to sea-level pressure (as used in weather data)
			// P: absolute pressure (mbar)
			// A: current altitude (meters)
			// returns sealevel pressure in mbar

		double altitude(double P, double P0);
			// convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
			// P: absolute pressure (mbar)
			// P0: fixed baseline pressure (mbar)
			// returns signed altitude in meters

		
	private:
	

		char readUInt(char address, uint16_t &value);
			// read an unsigned int (16 bits) from  MS5611 registers
			// address: BMP180 register address
			// value: external unsigned int for returned value (16 bits)
			// returns 1 for success, 0 for fail, with result in value

		char readBytes(uint32_t *values, char length);
			// read a number of bytes from MS5611 registers
			// values: array of char with register address in first location [0]
			// length: number of bytes to read back
			// returns 1 for success, 0 for fail, with read bytes in values[] array
			
		char writeBytes(uint8_t value);//, char length);
			// write/send a byte command to MS5611 designed register 
			// value: register address which will receive command
			// returns 1 for success, 0 for fail
			
		uint16_t C1,C2,C3,C4,C5,C6; 
		//variables which will receive factory calibration
		double Tref, TEMPSENS, OFFt1, SENSt1, dT;
		
};

#define MS5611_ADDRESS       (0x77)

#define MS5611_ADC_READ      (0x00)
#define MS5611_RESET         (0x1E)
#define MS5611_D1            (0x40)
#define MS5611_D2            (0x50)


#endif
