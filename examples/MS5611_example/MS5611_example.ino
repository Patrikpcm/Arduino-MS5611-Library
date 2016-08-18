/* MS5611 altitude example sketch

This sketch shows how to use the MS5611 pressure sensor
as an altimiter.
http://www.hpinfotech.ro/MS5611-01BA03.pdf

Like most pressure sensors, the MS5611 measures absolute pressure.
Since absolute pressure varies with altitude, you can use the pressure
to determine your altitude.

Because pressure also varies with weather, you must first take a pressure
reading at a known baseline altitude. Then you can measure variations
from that pressure

Hardware connections:

- (GND) to GND
+ (VDD) to 3.3V

(WARNING: do not connect + to 5V or the sensor will be damaged!)

You will also need to connect the I2C pins (SCL and SDA) to your
Arduino. The pins are different on different Arduinos:

Any Arduino pins labeled:  SDA  SCL
Uno, Redboard, Pro:        A4   A5
Mega2560, Due:             20   21
Leonardo:                   2    3

The MS5611 library uses floating-point equations based on instructions 
of PDF datashet.
http://www.hpinfotech.ro/MS5611-01BA03.pdf

Like Mike Grusin from SparkFun says, this code uses the "beerware" license. You can redistribute it and/or modify
it under the term of pay me a beer someday, case you find it useful.

*/

// Your sketch must #include this library, and the Wire library.
// (Wire is a standard library included with Arduino.):

#include <MS5611.h>
#include <Wire.h>

// You will need to create an MS5611 object, here called "pressure":

MS5611 pressure;
#define ALTITUDE 934.0 // Altitude of my house (Campo Magro), CO. in meters

void setup(){
  Serial.begin(9600);
  Serial.println("REBOOT");
  // Initialize the sensor (it is important to load factory calibration values stored on the PROM device).

  if (pressure.begin())
    Serial.println("MS5611 init success");
  else{
    // Something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("MS5611 init fail\n\n");
    while(1); // Pause forever.
  }
}

void loop(){

  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature(4);
  if (status != 0){

    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Second variable (1) indicates that function will compensate her math for low temperatures.
    // Use 1 = Compensate or 0 = Not compensate. For more information read the datashet pdf.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T, 1);
    if (status != 0){
      // Print out the measurement:
      Serial.print("Temperature: ");
      Serial.print(T,2);
      Serial.print("  °C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" °F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 4 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(4);
      if (status != 0){

        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // Third variable (1) indicates that function will compensate her math for low temperatures.
        // Use 1 = Compensate or 0 = Not compensate. For more information read the datashet pdf.
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T,1);
        if (status != 0){
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // i'm at 934 meters (Campo Magro, PR, BR)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  delay(5000);  // Pause for 5 seconds.
}
