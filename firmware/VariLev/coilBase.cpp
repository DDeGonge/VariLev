#include "coilBase.h"

VariLev::VariLev(unsigned int coils[4])
{
 // Initialize coils
 for(unsigned int i = 0; i < 4; i++)
 {
   pinMode(coils[i], OUTPUT);
   analogWrite(coils[i], 0);
 }
 coil_x_n = coils[0];
 coil_x_p = coils[1];
 coil_y_n = coils[2];
 coil_y_p = coils[3];

 // Configure some PID controller stuff
 x_controller.SetOutputLimits(0, 1.0);
 y_controller.SetOutputLimits(0, 1.0);
 z_controller.SetOutputLimits(0, 1.0);
}

bool VariLev::update_current_mags(int x, int y, int z)
{
 x_position = x;
 y_position = y;
 z_position = z;

 compute_pid();
 update_outputs();

 return true;
}

bool VariLev::set_mags_target(int x, int y, int z)
{
 x_target = x;
 y_target = y;
 z_target = z;
 return true;
}

bool VariLev::get_mags_target(int &x, int &y, int &z)
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
 calculate_pwm(y_power, yn_pwm, yp_pwm);
 xn_pwm *= (1 / z_power);
 xp_pwm *= (1 / z_power);
 yn_pwm *= (1 / z_power);
 yp_pwm *= (1 / z_power);

 analogWrite(coil_x_n, xn_pwm);
 analogWrite(coil_x_p, xp_pwm);
 analogWrite(coil_y_n, yn_pwm);
 analogWrite(coil_y_p, yp_pwm);

 // DEBUGGING ONLY
 Serial.print("xn: ");
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
 pwm_neg = percent <= 0.5 ? pwm_max : pwm_min + (percent / 0.5)*(pwm_max - pwm_min);
 pwm_pos = percent >= 0.5 ? pwm_max : pwm_max - ((percent - 0.5) / 0.5)*(pwm_max - pwm_min);
 return true;
}
