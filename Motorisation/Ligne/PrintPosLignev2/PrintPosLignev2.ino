#include <QTRSensors.h>

// This example is designed for use with eight RC QTR sensors. These
// reflectance sensors should be connected to digital pins 3 to 10. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The main loop of the example reads the raw sensor values (uncalibrated). You
// can test this by taping a piece of 3/4" black electrical tape to a piece of
// white paper and sliding the sensor across it. It prints the sensor values to
// the serial monitor as numbers from 0 (maximum reflectance) to 2500 (minimum
// reflectance; this is the default RC timeout, which can be changed with
// setTimeout()).

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t tempsorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A15, A14, A13, A12, A11, A10, A9, A8}, SensorCount);
  qtr.setEmitterPin(47);

  Serial.begin(9600);
}

void loop()
{
  // read raw sensor values
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 2500, where 0 means maximum
  // reflectance and 2500 means minimum reflectance

  boolean vide = true;
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    if (sensorValues[i] > 1600){
      
      Serial.print("|-|-|-|");
      sensorValues[i] = 1;
      vide = false;
    } else {
      sensorValues[i] = 0;
    }
    Serial.print('\t');
  }

  if (vide) {
    while (vide) {
        qtr.read(sensorValues);
        // print the sensor values as numbers from 0 to 2500, where 0 means maximum
        // reflectance and 2500 means minimum reflectance
        vide = true;
        boolean oukele = (tempsorValues[0] + tempsorValues[1] > tempsorValues[7] + tempsorValues[6]);
        if (oukele){
           Serial.print("<----");
        }
        for (uint8_t i = 0; i < SensorCount; i++)
        {
          if (sensorValues[i] > 1600){
            sensorValues[i] = 1;
            vide = false;
          } else {
            sensorValues[i] = 0;
          }
          Serial.print('\t');
        }
        if (not oukele) {
           Serial.print("---->");
        }
      Serial.println();
      delay(50); 
    }
  } else {
    for (uint8_t i = 0; i < SensorCount; i++)
    {
       tempsorValues[i] = sensorValues[i];        // Copie la liste sensorValues de manière débile
    }
  }
  Serial.println();
  
  delay(50);
}
