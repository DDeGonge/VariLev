#include "coilBase.h"

Coil::Coil(unsigned int dirPin0, unsigned int enPin0, unsigned int dirPin1, unsigned int enPin1, bool invert0, bool invert1)
{
  flip0 = invert0;
  flip1 = invert1;
  
  set_pins(dirPin0, enPin0, dirPin1, enPin1);
  set_power(0);
}

bool Coil::run_cal(Tlv493d &MagSensor, float caltime_s)
{
  // First find neutral field strength (when electromagnet is off)
  float delaytime_ms = 1000 / SENSORFREQ;
  int samples = int(0.1 * SENSORFREQ);
  float xmeasured = 0, ymeasured = 0, zmeasured = 0;
  for (int s = 0; s < samples; s++)
  {
    MagSensor.updateData();
    xmeasured += MagSensor.getX();
    ymeasured += MagSensor.getY();
    zmeasured += MagSensor.getZ();
    delay(delaytime_ms);
  }
  float x_zero = xmeasured / samples;
  float y_zero = ymeasured / samples;
  float z_zero = zmeasured / samples;

  // Then run all testpoints
  for (int i = 0; i < (sizeof(cal_powers) / sizeof(cal_powers[0])); i++)
  {
    set_power(cal_powers[i]);
    delay(5);
    int t_start = millis();
    int samples = (int)(caltime_s * SENSORFREQ);
    float xmeasured = 0, ymeasured = 0, zmeasured = 0;
    for (int s = 0; s < samples; s++)
    {
      MagSensor.updateData();
      xmeasured += (MagSensor.getX() - x_zero);
      ymeasured += (MagSensor.getY() - y_zero);
      zmeasured += (MagSensor.getZ() - z_zero);
      delay(delaytime_ms);
    }
    float multiplier = 1.0;
    xcal[i] = multiplier * xmeasured / samples;
    ycal[i] = multiplier * ymeasured / samples;
    zcal[i] = multiplier * zmeasured / samples;
    Serial.print("CalStep: ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(cal_powers[i]);
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

bool Coil::set_pins(unsigned int dirPin0, unsigned int enPin0, unsigned int dirPin1, unsigned int enPin1)
{
  dir_pin0 = dirPin0;
  en_pin0 = enPin0;
  dir_pin1 = dirPin1;
  en_pin1 = enPin1;
  pinMode(dir_pin0, OUTPUT);
  pinMode(en_pin0, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  pinMode(en_pin1, OUTPUT);
  return true;
}

bool Coil::set_power(int power)
{
  if ((power < -255) || (power > 255))
  {
    Serial.print("WARN, invalid power value :");
    Serial.println(power);
    return false;
  }
  else if (power == 0)
  {
    coil_power = 0;
    analogWrite(en_pin0, 0);
    analogWrite(en_pin1, 0);
    return true;
  }
  else
  {
    coil_power = power;
    bool dir0 = flip0 ? LOW : HIGH;
    bool dir1 = flip1 ? HIGH : LOW;
    if (power < 0)
    {
      dir0 = !dir0;
      dir1 = !dir1;
      power *= -1;
    }
    digitalWrite(dir_pin0, dir0);
    digitalWrite(dir_pin1, dir1);
    analogWrite(en_pin0, power);
    analogWrite(en_pin1, power);
    return true;
  }
}

bool Coil::get_distortion(double &x, double &y, double &z)
{
  double xd_new, yd_new, zd_new;
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
    xd_new = xcal[0];
    yd_new = ycal[0];
    zd_new = zcal[0];
  }
  else if (l_index >= 0)
  {
    float interp_percent = coil_power - cal_powers[l_index];
    interp_percent /= (cal_powers[l_index + 1] - cal_powers[l_index]);
    xd_new = xcal[l_index] + interp_percent * (xcal[l_index + 1] - xcal[l_index]);
    yd_new = ycal[l_index] + interp_percent * (ycal[l_index + 1] - ycal[l_index]);
    zd_new = zcal[l_index] + interp_percent * (zcal[l_index + 1] - zcal[l_index]);
  }
  else
  {
    return false;
  }
  xdist += lpf_cal * (xd_new - xdist);
  ydist += lpf_cal * (yd_new - ydist);
  zdist += lpf_cal * (zd_new - zdist);

  x -= xdist;
  y -= ydist;
  z -= zdist;
  return true;
}

VariLev::VariLev(Coil coils[4])
{
 // Initialize coils
 coil_x = coils[0];
 coil_y = coils[1];

 // Configure some PID controller stuff
 x_controller.SetOutputLimits(-1.0, 1.0);
 y_controller.SetOutputLimits(-1.0, 1.0);
 z_controller.SetOutputLimits(0, 1.0);

 int sampletime = int(1000000 / CYCLEFREQ);
 x_controller.SetSampleTime(sampletime);
 y_controller.SetSampleTime(sampletime);
 z_controller.SetSampleTime(sampletime);

 x_controller.Start(0, 0, 0);
 y_controller.Start(0, 0, 0);
 z_controller.Start(0, 0, 20);
}

//double VariLev::input_correction(double pos)
//{
//  double abspos = abs(pos);
//  int sign = pos / abspos;
//  if (abspos < 1.0)
//  {
//    return (1.0 * sign * abspos);
//  }
//  else
//  {
//    return (sign * (2 * sqrt(abspos) - 1));
//  }
//}

double VariLev::input_correction(double pos)
{
  double abspos = abs(pos);
  int sign = pos / abspos;
  if (abspos < 0.63)
  {
    return (1.0 * sign * pow(abspos, 2));
  }
  else
  {
    return (sign * (2 * sqrt(abspos) - 1.19));
  }
}

void VariLev::tip_correction(double &xtc, double &ytc)
{
  double xgoal = pow(x_power, 2) * deflection_mult;
  xgoal = fmax(fmin(xgoal, deflection_max), -deflection_max);
  double xchange = fmin(abs(xgoal - xDeflect), maxTipDelta);
  if (xDeflect < xgoal) xDeflect += xchange;
  else if (xDeflect > xgoal) xDeflect -= xchange;

  double ygoal = pow(y_power, 2) * deflection_mult;
  ygoal = fmax(fmin(ygoal, deflection_max), -deflection_max);
  double ychange = fmin(abs(ygoal - yDeflect), maxTipDelta);
  if (yDeflect < ygoal) yDeflect += ychange;
  else if (yDeflect > ygoal) yDeflect -= ychange;

  xtc -= xDeflect;
  ytc -= yDeflect;
}

void VariLev::update_tip_parameters(double rise, double powmult, double maxpow)
{
  risetime_s = rise;
  deflection_max = maxpow;
  deflection_mult = powmult;
  maxTipDelta = deflection_max / (risetime_s * SENSORFREQ);
}

bool VariLev::update_current_mags(double x, double y, double z)
{
//  if (DEBUGPRINT)
//  {
//    Serial.print("pre-dist:");
//    Serial.print(x);
//  }
  // Compensate for current coil outputs
  double ynull = 0, xnull = 0;
  coil_x.get_distortion(x, ynull, z);
  coil_y.get_distortion(xnull, y, z);

//  if (DEBUGPRINT)
//  {
//    Serial.print("\tpost-dist:");
//    Serial.print(x);
//  }

//  tip_correction(x, y);

  // Normalize
  double xn = x; // input_correction(x);
  double yn = y; // input_correction(y);

  // Update positions with LPF
  x_position += lpf_mult * (xn - x_position);
  y_position += lpf_mult * (yn - y_position);
//  double new_z_mm;
//  z_mag_to_mm(z, new_z_mm);
//  z_position += lpf_mult * (new_z_mm - z_position);

  // Debug print
//  Serial.print("COMMANDS: \t");
//  Serial.print(x_position);
//  Serial.print("\t ; ");
//  Serial.print(y_position);
//  Serial.print("\t ; ");
//  Serial.println(z_position);

  // Do the control loop stuff
  double new_x_power = x_controller.Run(x_position);
  double new_y_power = y_controller.Run(y_position);
//  double new_z_power = z_controller.Run(z_position);

  // idk if this will help or not
  x_power = (fabs(new_x_power) / new_x_power) * sqrt(fabs(new_x_power));
  y_power = (fabs(new_y_power) / new_y_power) * sqrt(fabs(new_y_power));
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
  coil_x.set_power((int)(x_power * 255));
  coil_y.set_power((int)(y_power * 255));

 if (DEBUGPRINT)
 {
   Serial.print("\tposition:");
   Serial.print(x_position);
   Serial.print("\tpower:");
   Serial.print(x_power);
   Serial.print("\tdeflect:");
   Serial.print(xDeflect);
   Serial.print("\tkp:");
   Serial.print(x_controller.GetLastP(), 3);
   Serial.print("\tki:");
   Serial.print(x_controller.GetLastI(), 3);
   Serial.print("\tkd:");
   Serial.print(x_controller.GetLastD(), 3);
   Serial.println();
 }
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
