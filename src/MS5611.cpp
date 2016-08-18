/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.
(c) 2016 Patrik Luiz Gogola - Campo Magro - Paraná - Brasil

Version: 1.0.0 29/07/2016

USES FLOATING-POINT EQUATIONS.

Forked from MS5611 library by Korneliusz Jarzebski and SFE_BMP180 library by Mike Grusin from SparkFun Electronics.

Like Mike Grusin from SparkFun says, this code uses the "beerware" license. You can redistribute it and/or modify
it under the term of pay me a beer someday, case you find it useful.
*/

#include <MS5611.h>
#include <Wire.h>
#include <stdio.h>
#include <math.h>

MS5611::MS5611(){
//Base library type
}

char MS5611::begin(){
//initialize library for subsequent pressure and temperature measurement
    
    Wire.begin();
    // Start up the Arduino's "Wire" (I2C) library

    // The MS5611 includes factory calibration data stored on the PROM device.
    // Each device has different numbers, these must be retrieved and
    // used in the calculations when taking pressure and temperature measurements.

    // To do it,the reset sequence shall be sent once after power-on to make sure 
    // that the calibration PROM gets loaded into the internal register.

    // Let's do this and also retrieve calibration data from device.
    if ( reset() &&
         readUInt(0xA2,C1) && 
         readUInt(0xA4,C2) && 
         readUInt(0xA6,C3) && 
         readUInt(0xA8,C4) && 
         readUInt(0xAA,C5) && 
         readUInt(0xAC,C6) ) {

        // All reads completed successfully!

        // If you need to check your math using known numbers,
        // you can uncomment one of these examples.
        // (The correct results are commented in the below functions.)

        // According MS5611-01BA01 pdf, the typical results revolve around these values: 
        // C1 = 40127; C2 = 36924; C3 = 23317; C4 = 23282; C5 = 33464; C6 = 28312;
        
        /*
        Serial.print("AC1: "); Serial.println(AC1);
        Serial.print("AC2: "); Serial.println(AC2);
        Serial.print("AC3: "); Serial.println(AC3);
        Serial.print("AC4: "); Serial.println(AC4);
        Serial.print("AC5: "); Serial.println(AC5);
        Serial.print("AC6: "); Serial.println(AC6);
        Serial.print("VB1: "); Serial.println(VB1);
        Serial.print("VB2: "); Serial.println(VB2);
        Serial.print("MB: "); Serial.println(MB);
        Serial.print("MC: "); Serial.println(MC);
        Serial.print("MD: "); Serial.println(MD);
        */
        
        // Initial compute floating-point polynominals:

        Tref = (C5 * 256.0);
        TEMPSENS = (C6 / 8388608.0);
        OFFt1 = (C2 * 65536.0);
        SENSt1 = (C1 * 32768.0);
        
        /*
        Serial.println();
        Serial.print("C1: "); Serial.println(C1);
        Serial.print("C2: "); Serial.println(C2);
        Serial.print("C3: "); Serial.println(C3);
        Serial.print("C4: "); Serial.println(C4);
        Serial.print("C5: "); Serial.println(C5);
        Serial.print("C6: "); Serial.println(C6);
        Serial.print("Tref: "); Serial.println(Tref);
        Serial.print("TEMPSENS: "); Serial.println(TEMPSENS, 10);
        Serial.print("OFFt1: "); Serial.println(OFFt1);
        Serial.print("SENSt1: "); Serial.println(SENSt1);
        */
        
        return(1);
        // Success!
    }
    else // Error reading calibration data; bad component or connection?
        return(0);
    
}

char MS5611::reset(){
    // Send the RESET sequence to sensor, this is necessary because MS5611
    // includes factory calibration data stored on the PROM device which needs to
    // loaded into the internal register. 
		
        Wire.beginTransmission(MS5611_ADDRESS);
        Wire.write(MS5611_RESET);
        Wire.endTransmission();
        delay (20); //The Reset sequence needs a little delay to work well

        if( Wire.endTransmission() == 0)
            return(1);
        else
        // Error to start transmission; bad component or connection?
            return(0);
}


char MS5611::readUInt(char address, uint16_t &value){
    // Read an unsigned integer (two bytes) from device
    // address: register to start reading (plus subsequent register)
    // value: external variable to store data (function modifies value)
    
    uint32_t data[2];

    data[0] = address;
    if (readBytes(data,2)){
        value = (((uint16_t)data[0]<<8)|(uint16_t)data[1]);
        return(1);
    }
    value = 0;
    return(0);
}

char MS5611::writeBytes(uint8_t value){
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
 
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(value);

    if ( Wire.endTransmission() == 0)
    	return(1);
    else
    	return(0);
}

char MS5611::readBytes(uint32_t *values, char length){
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(values[0]);
   
    if (Wire.endTransmission() == 0){
        
        Wire.requestFrom(MS5611_ADDRESS, length);

        while(Wire.available() != length); // wait until bytes are ready
        
        for(byte x=0; x<length; x++){
    	    values[x] = Wire.read();
         //	Serial.print("X: "); Serial.println(values[x]);
        }
        return(1);
    }
    else
        return(0);
}

char MS5611::startTemperature(char oversampling){
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
    unsigned char delay, accuracy;

       switch (oversampling){

        case 0:
            delay = 3;
            accuracy = MS5611_D2; //(OSR=256 -> 0x50)
        break;
        case 1:
            delay = 5;
            accuracy = MS5611_D2 + 0x02; //(OSR=512 -> 0x52)
        break;
        case 2:
            delay = 8;
            accuracy = MS5611_D2 + 0x04; //(OSR=1024 -> 0x54)
        break;
        case 3:
            delay = 15;
            accuracy = MS5611_D2 + 0x06; //(OSR=2048 -> 0x56)
        break;
        case 4:
            delay = 20;
            accuracy = MS5611_D2 + 0x08; //(OSR=4096 -> 0x58)
        break;
        default:
            delay = 3;
            accuracy = MS5611_D2; //(OSR=256 -> 0x50)
        break;
    }
   
    if (writeBytes(accuracy)) // good write?
        return(delay); // return the delay in ms (rounded up) to wait before retrieving data
    else
        return(0); // or return 0 if there was a problem communicating with the BMP
}


char MS5611::getTemperature(double &T, byte compensate){
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.

    uint32_t data[3];
    double D2;
    
    data[0] = MS5611_ADC_READ;
   
    if (readBytes(data, 3)) { // good read, calculate temperature
        
        D2 = ( (data[0] << 16) | (data[1] << 8) | data[2] );
        //Operator OR for bits help us to group the bytes in a single variable.
        dT = (D2 - Tref);
        T = ((2000.0 + (dT * TEMPSENS))/100);
        
        //MS5611 have a second and a third order math to compensate temperature and pressure in low temperature ambients.
        if (compensate == 1 && T < 20.0) 
        	//This math increase temperature and pressure accuracy.
        	T = (T - ((dT*dT) / (2147483648)));
        
        /*
        Serial.println();
        Serial.print("D2: data[0]"); Serial.println(data[0], BIN);
        Serial.print("D2: data[1]"); Serial.println(data[1], BIN);
        Serial.print("D2: data[2]"); Serial.println(data[2], BIN);
        Serial.print("D2: "); Serial.printn(D2); //Serial.print("   BIN: "); Serial.println((D2D, BIN); //create a variable uint32_t D2D to receive OR of bits to test it
        Serial.print("dT: "); Serial.println(dT);                                                          
        Serial.print("T: "); Serial.println(T);
        */
        return(1);
    }
    else
        return 0;
}


char MS5611::startPressure(char oversampling){
// Begin a pressure reading.
// Oversampling: 0 to 6, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.

    unsigned char delay, accuracy; //result;
        
    switch (oversampling){

        case 0:
            delay = 3;
            accuracy = MS5611_D1; //(OSR=256 -> 0x40)
        break;
        case 1:
            delay = 5;
          	accuracy = MS5611_D1 + 0x02; //(OSR=512 -> 0x42);
        break;
        case 2:
            delay = 8;
           	accuracy = MS5611_D1 + 0x04; //(OSR=1024 -> 0x44);
        break;
        case 3:
            delay = 15;
        accuracy = MS5611_D1 + 0x06; //(OSR=2048 -> 0x46);
        break;
        case 4:
            delay = 20;
           	accuracy = MS5611_D1 + 0x08; //(OSR=4096 -> 0x48);
        break;
        default:
            delay = 3;
        	accuracy = MS5611_D1; //(OSR=256 -> 0x40);
        break;
    }

    if (writeBytes(accuracy)) // good write?
        return(delay); // return the delay in ms (rounded up) to wait before retrieving data
    else
        return(0); // or return 0 if there was a problem communicating with the BMP
}

char MS5611::getPressure(double &P, double &T, byte compensate){
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().

    //unsigned char data[3];
    //char result;
    double D1, OFF, SENS;
    uint32_t data[3];

    data[0] = MS5611_ADC_READ;

    //result = readBytes(data, 3);
    if (readBytes(data, 3)){
    // good read, calculate pressure
        D1 = ( (data[0] << 16) | (data[1] << 8) | data[2] );
        //Operator OR for bits helps us to group the bytes in a single variable.
        
        OFF = OFFt1 + (C4 * dT) / 128.0;
        SENS = SENSt1 + (C3 * dT) / 256.0;
        
        if(compensate == 1){ //if you want to compensate pressure, I suggest a temperature compensate also.
        	OFF = OFF - (5 * (pow((T - 2000),2) / 2));
        	SENS = SENS - (5 * (pow((T - 2000),2) / 4));
        	P = (((D1 * SENS / 2097152.0 - OFF)/32768.0) / 100);
        }
		else
			P = (((D1 * SENS / 2097152.0 - OFF)/32768.0) / 100);
		     
        //pressure compensate have a third order math to compensate pressure in very low temperatures (temperatures < -15.0).
        //as in my country we don't have temperatures like this, the code as been commited.
        /*if (compensate == 'S' && T < (-15.0)){ 
        	OFF = OFF - (OFF + 7 * pow((T + 1500), 2));
        	SENS = SENS - (SENS + 11 * pow((T + 1500), 2) / 2);
        	P = (((D1 * SENS / 2097152.0 - OFF)/32768.0) / 100);
        }*/

        
       /* Serial.println();
        Serial.print("D1: data[0]"); Serial.println(data[0], BIN);
        Serial.print("D1: data[1]"); Serial.println(data[1], BIN);
        Serial.print("D1: data[2]"); Serial.println(data[2], BIN);
        Serial.print("D1: "); Serial.println(D1); // Serial.print("   BIN:"); Serial.println(D1D, BIN); //creat a variable uint32_t D1D to receive OR of bits to test it
        Serial.print("OFF: "); Serial.println(OFF);
        Serial.print("SENS: "); Serial.println(SENS);
        Serial.print("T: "); Serial.println(T);
        Serial.print("P: "); Serial.println(P);
        */
        return(1);
    }
    else
        return(0);
}

double MS5611::sealevel(double P, double A){
// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.

    return(P/pow(1-(A/44330.0),5.255));
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::altitude(double P, double P0){
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// Return altitude (meters) above baseline.

    return (44330.0 * (1.0 - pow(P / P0, 0.190294957183635)));
}