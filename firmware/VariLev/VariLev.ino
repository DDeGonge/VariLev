#include "coilBase.h"
#include "pins.h"


void setup() {
  Serial.begin(250000);
  while(!Serial);
}

void loop() {
  Serial.println("INITIALIZING");
  // x-, x+, y-, y+
  Coil coils[4] = {Coil(coil2), Coil(coil0), Coil(coil3), Coil(coil1)};

  Tlv493d MagSensor = Tlv493d();
  MagSensor.begin();
  Serial.println("SENSOR STARTED");
//  MagSensor.setAccessMode(MagSensor.FASTMODE);
  MagSensor.setAccessMode(MagSensor.LOWPOWERMODE);
  MagSensor.disableTemp();


  // STATIC TEST
//  while(true)
//  {
//    for (int c = 0; c < 4; c++)
//    {
//      coils[c].set_power(255);
//    }
//    Serial.println("on");
//    delay(2000);
//    Serial.println("off");
//    for (int c = 0; c < 4; c++)
//    {
//      coils[c].set_power(0);
//    }
//    delay(2000);
//  }

  double mag_x, mag_y, mag_z;
  while(true)
  {
    coils[0].set_power(0);
    coils[1].set_power(0);
    coils[2].set_power(255);
    coils[3].set_power(0);
    for (int i = 0; i < 1000; i++)
    {
      MagSensor.updateData();
      mag_x = MagSensor.getX();
      mag_y = MagSensor.getY();
      mag_z = MagSensor.getZ();
    
      Serial.print(mag_x);
      Serial.print("\t ; ");
      Serial.print(mag_y);
      Serial.print("\t ; ");
      Serial.println(mag_z);
    }
    coils[0].set_power(255);
    coils[1].set_power(0);
    coils[2].set_power(0);
    coils[3].set_power(0);
    for (int i = 0; i < 1000; i++)
    {
      MagSensor.updateData();
      mag_x = MagSensor.getX();
      mag_y = MagSensor.getY();
      mag_z = MagSensor.getZ();
    
      Serial.print(mag_x);
      Serial.print("\t ; ");
      Serial.print(mag_y);
      Serial.print("\t ; ");
      Serial.println(mag_z);
    }
  }
  
  // RAMP TEST
//  while(true)
//  {
//    int ramp_min = 180;
//    int dwell = 100;
//    for (int i = ramp_min; i < 255; i++)
//    {
//      coils[0].set_power(i);
//      coils[1].set_power(ramp_min + (255 - i));
//      Serial.println(i);
//      delay(dwell);
//    }
//    for (int i = 255; i > ramp_min; i--)
//    {
//      coils[0].set_power(i);
//      coils[1].set_power(ramp_min + (255 - i));
//      Serial.println(i);
//      delay(dwell);
//    }
//  }

  // Calibrate coils
//  Serial.println("CALIBRATING");
//  for (int c = 0; c < 4; c++)
//  {
//    coils[c].run_cal(MagSensor, 0.2);
//  }

  VariLev LevObj = VariLev(coils);
  LevObj.enable_controllers();
  LevObj.set_mags_target(-1.0, 0.0, 100.0);

//  double mag_x, mag_y, mag_z;

  while(true)
  {
    delay(MagSensor.getMeasurementDelay());
    MagSensor.updateData();
    mag_x = MagSensor.getX();
    mag_y = MagSensor.getY();
    mag_z = MagSensor.getZ();
  
    Serial.print(mag_x);
    Serial.print("\t ; ");
    Serial.print(mag_y);
    Serial.print("\t ; ");
    Serial.println(mag_z);
    LevObj.update_current_mags(mag_x, mag_y, mag_z);
  }
}
