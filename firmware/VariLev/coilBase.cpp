#include "coilBase.h"

Coil::Coil(unsigned int dirPin, unsigned int enPin, bool invert_flag)
{
  if (invert_flag)  invert = -1;
  else              invert =  1;
  
  set_pins(dirPin, enPin);
  set_power(0);
}

bool Coil::run_cal(Tlv493d &MagSensor, float caltime_s)
{
  // First find neutral field strength (when electromagnet is off)
  float delaytime_ms = MagSensor.getMeasurementDelay();
  float xmeasured = 0, ymeasured = 0, zmeasured = 0;
  for (int s = 0; s < 100; s++)
  {
    MagSensor.updateData();
    xmeasured += MagSensor.getX();
    ymeasured += MagSensor.getY();
    zmeasured += MagSensor.getZ();
    delay(delaytime_ms);
  }
  float x_zero = xmeasured / 100;
  float y_zero = ymeasured / 100;
  float z_zero = zmeasured / 100;

  // Then run all testpoints
  for (int i = 0; i < (sizeof(cal_powers) / sizeof(cal_powers[0])); i++)
  {
    set_power(cal_powers[i]);
    delay(5);
    int t_start = millis();
    int samples = (int)(caltime_s * (1000 / delaytime_ms));
    float xmeasured = 0, ymeasured = 0, zmeasured = 0;
    for (int s = 0; s < samples; s++)
    {
      MagSensor.updateData();
      xmeasured += (MagSensor.getX() - x_zero);
      ymeasured += (MagSensor.getY() - y_zero);
      zmeasured += (MagSensor.getZ() - z_zero);
      delay(delaytime_ms);
    }
    xcal[i] = xmeasured / samples;
    ycal[i] = ymeasured / samples;
    zcal[i] = zmeasured / samples;
    Serial.print("CalStep: ");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(xcal[i]);
    Serial.print(" ");
    Serial.print(ycal[i]);
    Serial.print(" ");
    Serial.println(zcal[i]);
  }
  set_power(0);
  calibrated = true;
  return true;
}

bool Coil::set_pins(unsigned int dirPin, unsigned int enPin)
{
  dir_pin = dirPin;
  en_pin = enPin;
  pinMode(dir_pin, OUTPUT);
  pinMode(en_pin, OUTPUT);
  return true;
}

bool Coil::set_power(int power)
{
  power *= invert;
  if ((power < -255) || (power > 255))
  {
    Serial.print("WARN, invalid power value :");
    Serial.println(power);
    return false;
  }
  else if (power == 0)
  {
    coil_power = 0;
    analogWrite(en_pin, 0);
    return true;
  }
  else if (power < 0)
  {
    coil_power = power;
    digitalWrite(dir_pin, LOW);
    analogWrite(en_pin, -power);
    return true;
  }
  else if (power > 0)
  {
    coil_power = power;
    digitalWrite(dir_pin, HIGH);
    analogWrite(en_pin, power);
    return true;
  }
  else return false;
}

bool Coil::get_distortion(double &x, double &y, double &z)
{
  if (!calibrated)
  {
    Serial.println("ERROR: No calibration. Can't get distortion");
    return false;
  }

  // find lower index
  int l_index = -2;
  for (int i = 0; i < (sizeof(cal_powers) / sizeof(cal_powers[0])); i++)
  {
    if (coil_power <= cal_powers[i])
    {
      l_index = i - 1;
      break;
    }
  }
  if (l_index == -1)
  {
    x -= xcal[0];
    y -= ycal[0];
    z -= zcal[0];
  }
  else if (l_index >= 0)
  {
    float interp_percent = coil_power - cal_powers[l_index];
    interp_percent /= (cal_powers[l_index + 1] - cal_powers[l_index]);
    x -= xcal[l_index] + interp_percent * (xcal[l_index + 1] - xcal[l_index]);
    y -= ycal[l_index] + interp_percent * (ycal[l_index + 1] - ycal[l_index]);
    z -= zcal[l_index] + interp_percent * (zcal[l_index + 1] - zcal[l_index]);
  }
  else
  {
    return false;
  }
  return true;
}

VariLev::VariLev(Coil coils[4])
{
 // Initialize coils
 coil_xn = coils[0];
 coil_xp = coils[1];
 coil_yn = coils[2];
 coil_yp = coils[3];

 // Configure some PID controller stuff
 x_controller.SetOutputLimits(-1.0, 1.0);
 y_controller.SetOutputLimits(-1.0, 1.0);
 z_controller.SetOutputLimits(0, 1.0);

 x_controller.SetSampleTime(10);
 y_controller.SetSampleTime(10);
 z_controller.SetSampleTime(10);

 x_controller.Start(0, 0, 0);
 y_controller.Start(0, 0, 0);
 z_controller.Start(0, 0, 20);
}

bool VariLev::update_current_mags(double x, double y, double z)
{
  // Compensate for current coil outputs
  coil_xn.get_distortion(x, y, z);
  coil_xp.get_distortion(x, y, z);
  coil_yn.get_distortion(x, y, z);
  coil_yp.get_distortion(x, y, z);

  // DEBUGGING
  Serial.print("CAL OUTPUTS: \t");
  Serial.print("\tX:");
  Serial.print(x);
  Serial.print("\tY:");
  Serial.print(y);
  Serial.print("\tZ:");
  Serial.println(z);

  // Normalize
  double fieldTotal = pow(pow(x, 2) + pow(y, 2) + pow(z, 2), 0.5);
  double xn = x; // / fieldTotal;
  double yn = y; // / fieldTotal;

  // DEBUGGING
  Serial.print("NORMALIZED: \t");
  Serial.print("\tX:");
  Serial.print(xn);
  Serial.print("\tY:");
  Serial.println(yn);

  // Update positions with LPF
  x_position += lpf_mult * (xn - x_position);
  y_position += lpf_mult * (yn - y_position);
  double new_z_mm;
  z_mag_to_mm(z, new_z_mm);
  z_position += lpf_mult * (new_z_mm - z_position);

  // Debug print
  Serial.print("COMMANDS: \t");
  Serial.print(x_position);
  Serial.print("\t ; ");
  Serial.print(y_position);
  Serial.print("\t ; ");
  Serial.println(z_position);

  // Do the control loop stuff
  x_power = x_controller.Run(x_position);
  y_power = y_controller.Run(y_position);
  z_power = z_controller.Run(z_position);
  update_outputs();
  
  return true;
}

bool VariLev::set_mags_target(double x, double y, double z)
{
 double zpos = 0;
 z_mag_to_mm(z, zpos);
 x_controller.Setpoint(x);
 y_controller.Setpoint(y);
 z_controller.Setpoint(zpos);
 return true;
}

bool VariLev::get_mags_target(double &x, double &y, double &z)
{
 x = x_controller.GetSetpoint();
 y = y_controller.GetSetpoint();
 z = z_controller.GetSetpoint();
 return true;
}


bool VariLev::enable_controllers()
{
 x_controller.SetMode(AUTOMATIC);
 y_controller.SetMode(AUTOMATIC);
 z_controller.SetMode(AUTOMATIC);
 return true;
}

bool VariLev::disable_controllers()
{
 x_controller.SetMode(MANUAL);
 y_controller.SetMode(MANUAL);
 z_controller.SetMode(MANUAL);
 return true;
}

bool VariLev::update_outputs()
{
  // TODO handle Z somehow
  coil_xp.set_power((int)(x_power * 255));
  coil_xn.set_power((int)(x_power * -255));
  coil_yp.set_power((int)(y_power * 255));
  coil_yn.set_power((int)(y_power * -255));

   // DEBUGGING ONLY
   Serial.print("xpow: ");
   Serial.print(x_power);
   Serial.print("\typow: ");
   Serial.println(y_power);
   return true;
}

bool VariLev::z_mag_to_mm(double zmag, double &zdist)
{
  // Close enough for now anyway
  zdist = 100;
  zdist = zmag > 1 ? zdist / zmag : zdist;
  return true;
}

void VariLev::update_xy_tuning(double kp, double ki, double kv)
{
  x_controller.SetTunings(kp, ki, kv);
  y_controller.SetTunings(kp, ki, kv);
}
