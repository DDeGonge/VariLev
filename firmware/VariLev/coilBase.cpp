#include "coilBase.h"

Coil::Coil(unsigned int pin)
{
  set_pin(pin);
}

bool Coil::run_cal(Tlv493d &MagSensor, float caltime_s)
{
  float delaytime_ms = MagSensor.getMeasurementDelay();
  for (int i = 0; i < (sizeof(cal_powers) / sizeof(cal_powers[0])); i++)
  {
    set_power(cal_powers[i]);
    delay(10);
    int t_start = millis();
    int samples = (int)(caltime_s * (1000 / delaytime_ms));
    float xmeasured = 0, ymeasured = 0, zmeasured = 0;
    for (int s = 0; s < samples; s++)
    {
      MagSensor.updateData();
      xmeasured += MagSensor.getX();
      ymeasured += MagSensor.getY();
      zmeasured += MagSensor.getZ();
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

bool Coil::set_pin(unsigned int pin)
{
  coil_pin = pin;
  pinMode(coil_pin, OUTPUT);
  analogWrite(coil_pin, 0);
  return true;
}

bool Coil::set_power(unsigned int power)
{
  if (power < 0)
  {
    Serial.print("WARN, invalid set power for coil ");
    Serial.print(coil_pin);
    Serial.print(" : ");
    Serial.println(power);
    return false;
  }
  else
  {
    coil_power = power;
    analogWrite(coil_pin, power);
    return true;
  }
}

bool Coil::get_distortion(double &x, double &y, double &z)
{
  if (!calibrated)
  {
//    Serial.println("ERROR: No calibration. Can't get distortion");
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

    // DEBUGGING
    Serial.print("CAL OUTPUTS: \t");
    Serial.print("\tX:");
    Serial.print(x);
    Serial.print("\tY:");
    Serial.print(y);
    Serial.print("\tZ:");
    Serial.println(z);
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
 coil_x_n = coils[0];
 coil_x_p = coils[1];
 coil_y_n = coils[2];
 coil_y_p = coils[3];

 // Configure some PID controller stuff
 x_controller.SetOutputLimits(-0.5, 0.5);
 y_controller.SetOutputLimits(-0.5, 0.5);
 z_controller.SetOutputLimits(0, 1.0);
}

bool VariLev::update_current_mags(double x, double y, double z)
{
  // Compensate for current coil outputs
  coil_x_n.get_distortion(x, y, z);
  coil_x_p.get_distortion(x, y, z);
  coil_y_n.get_distortion(x, y, z);
  coil_y_p.get_distortion(x, y, z);

  // Update positions with LPF
  x_position += lpf_mult * (x - x_position);
  y_position += lpf_mult * (y - y_position);
  double new_z_mm;
  z_mag_to_mm(z, new_z_mm);
  z_position += lpf_mult * (new_z_mm - z_position);

  // Debug print
  Serial.print(x_position);
  Serial.print("\t ; ");
  Serial.print(y_position);
  Serial.print("\t ; ");
  Serial.println(z_position);

  // Do the control loop stuff
  compute_pid();
  update_outputs();
  
  return true;
}

bool VariLev::set_mags_target(double x, double y, double z)
{
 x_target = x;
 y_target = y;
 z_mag_to_mm(z, z_position);
 return true;
}

bool VariLev::get_mags_target(double &x, double &y, double &z)
{
 x = x_target;
 y = y_target;
 z = z_target;
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


bool VariLev::compute_pid()
{
 x_controller.Compute();
 y_controller.Compute();
 z_controller.Compute();
 return true;
}

bool VariLev::update_outputs()
{
 unsigned int xn_pwm, xp_pwm, yn_pwm, yp_pwm;
 calculate_pwm(x_power, xn_pwm, xp_pwm);
 calculate_pwm(-y_power, yn_pwm, yp_pwm);
// xn_pwm *= 2 * z_power;
// xp_pwm *= 2 * z_power;
// yn_pwm *= 2 * z_power;
// yp_pwm *= 2 * z_power;

 coil_x_n.set_power(xn_pwm);
 coil_x_p.set_power(xp_pwm);
 coil_y_n.set_power(yn_pwm);
 coil_y_p.set_power(yp_pwm);

 // DEBUGGING ONLY
 Serial.print("xpow: ");
 Serial.print(x_power);
 Serial.print("\typow: ");
 Serial.print(y_power);
 Serial.print("\tzpow: ");
 Serial.print(z_power);
 Serial.print("\txn: ");
 Serial.print(xn_pwm);
 Serial.print("\txp: ");
 Serial.print(xp_pwm);
 Serial.print("\tyn: ");
 Serial.print(yn_pwm);
 Serial.print("\typ: ");
 Serial.println(yp_pwm);
 return true;
}

bool VariLev::calculate_pwm(double percent, unsigned int &pwm_neg, unsigned int &pwm_pos)
{
 // This is gross, I'm sorry.
 pwm_neg = percent <= 0 ? pwm_max : pwm_max - (percent / 0.5)*(pwm_max - pwm_min);
 pwm_pos = percent >= 0 ? pwm_max : pwm_max + (percent / 0.5)*(pwm_max - pwm_min);
 return true;
}

bool VariLev::z_mag_to_mm(double zmag, double &zdist)
{
  // Close enough for now anyway
  zdist = 100;
  zdist = zmag > 1 ? zdist / zmag : zdist;
  return true;
}
