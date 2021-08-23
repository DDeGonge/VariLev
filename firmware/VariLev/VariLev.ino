#include "coilBase.h"
#include "pins.h"

void setup() {
  delay(200);
  Serial.begin(250000);
//  attachInterrupt(13, printtime, FALLING);  // INTERRUPT ON SCL
  while(!Serial);
}

void loop() {
  delay(100);
  Serial.println("INITIALIZING");
  // (x-, x+, y-, y+)
  Coil coils[2] = {
    Coil(coil3_DIR, coil3_EN, coil1_DIR, coil1_EN, false, true),
    Coil(coil0_DIR, coil0_EN, coil2_DIR, coil2_EN, false, true),
  };

  Wire.setClock(10000000);
  Tlv493d MagSensor = Tlv493d();
  MagSensor.begin();
  MagSensor.setAccessMode(MagSensor.FASTMODE); // LOWPOWERMODE FASTMODE
  MagSensor.disableTemp();
//  MagSensor.enableInterrupt();
  Serial.println("SENSOR STARTED");

  // DEBUG TESTS
//  while(true)
//  {
//    print_mag(MagSensor);
//    ramp_test(coils);
//    cycle_all(coils);
//  }

  // Calibrate coils
  Serial.println("CALIBRATING");
  for (int c = 0; c < 2; c++)
  {
    coils[c].run_cal(MagSensor, 0.1);
  }

  VariLev LevObj = VariLev(coils);
  LevObj.enable_controllers();
  LevObj.set_mags_target(0.0, 0.0, 100.0);

  double mag_x, mag_y, mag_z;
  double kp, ki, kv;
  int measureDelayus = (int)(1000000 / CYCLEFREQ);
  long nextMeasureTime = micros() + measureDelayus;

  while(true)
  {
    while(Serial.available() > 0)
    {
      kp = (double)Serial.parseFloat();
      ki = (double)Serial.parseFloat();
      kv = (double)Serial.parseFloat();
      char r = Serial.read();
      if(r == '\n'){}
    
      Serial.print("kp =  ");
      Serial.println(kp, 3);
      Serial.print("ki =  ");
      Serial.println(ki, 3);
      Serial.print("kv =  ");
      Serial.println(kv, 3);

      LevObj.update_xy_tuning(kp, ki, kv);
      nextMeasureTime = micros() + measureDelayus;
    }

    MagSensor.updateData();
    mag_x = MagSensor.getX();
    mag_y = MagSensor.getY();
    mag_z = MagSensor.getZ();
  
//    Serial.print(mag_x);
//    Serial.print("\t ; ");
//    Serial.print(mag_y);
//    Serial.print("\t ; ");
//    Serial.println(mag_z);
    LevObj.update_current_mags(mag_x, mag_y, mag_z);

    bool hitMark = false;
    while (micros() < nextMeasureTime) hitMark = true;

    if (!hitMark)
    {
      Serial.println("TIMING MISS, LOWER CYCLE FREQ");
      nextMeasureTime = micros() + measureDelayus;
    }
    else nextMeasureTime += measureDelayus;
  }
}

void print_mag(Tlv493d MagSensor)
{
//  delay(MagSensor.getMeasurementDelay());
  MagSensor.updateData();

  Serial.print(micros());
  Serial.print("\t");
  Serial.println(MagSensor.getX());

//  Serial.print(MagSensor.getX());
//  Serial.print("\t ; ");
//  Serial.print(MagSensor.getY());
//  Serial.print("\t ; ");
//  Serial.println(MagSensor.getZ());
}

void ramp_test(Coil dacoils[4])
{
  int ramp_min = 180;
  int dwell = 100;
  for (int i = ramp_min; i < 255; i++)
  {
    dacoils[0].set_power(i);
    dacoils[1].set_power(ramp_min + (255 - i));
    Serial.println(i);
    delay(dwell);
  }
  for (int i = 255; i > ramp_min; i--)
  {
    dacoils[0].set_power(i);
    dacoils[1].set_power(ramp_min + (255 - i));
    Serial.println(i);
    delay(dwell);
  }
}

void cycle_all(Coil dacoils[4])
{
  for (int c = 0; c < 4; c++)
  {
    Serial.println(c);
    dacoils[c].set_power(255);
    delay(1000);
    dacoils[c].set_power(0);
  }
}
