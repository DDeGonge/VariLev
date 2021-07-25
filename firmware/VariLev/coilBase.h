#include <PID_v1.h>
#include <Arduino.h>
#include <Tlv493d.h>

struct Coil
{
  Coil() {};
  Coil(unsigned int pin);

  public:
  bool run_cal(Tlv493d &MagSensor, float caltime_s);
  bool set_pin(unsigned int pin);
  bool set_power(unsigned int power);
  bool get_distortion(double &x, double &y, double &z);

  private:
  unsigned int coil_pin = 0;
  unsigned int coil_power = 0;
  unsigned int cal_powers[11] = {55, 75, 95, 115, 135, 155, 175, 195, 215, 235, 255};
  double xcal[11], ycal[11], zcal[11];
  bool calibrated = false;
  
};

struct VariLev
{
  VariLev(Coil coils[4]);
  // Coil initialization should be in order x-, x+, y-, y+
  
  public:
  
  bool update_current_mags(double x, double y, double z);
  bool set_mags_target(double x, double y, double z);
  bool get_mags_target(double &x, double &y, double &z);
  
  bool enable_controllers();
  bool disable_controllers();
  
  
  private:
  
  bool compute_pid();
  bool update_outputs();
  bool calculate_pwm(double percent, unsigned int &pwm_neg, unsigned int &pwm_pos);
  bool z_mag_to_mm(double zmag, double &zdist);
  
  Coil coil_x_n, coil_x_p, coil_y_n, coil_y_p;
  
  double x_position = 0;
  double y_position = 0;
  double z_position = 0;
  
  double x_power = 0.0;
  double y_power = 0.0;
  double z_power = 0.5;
  
  double x_target = 0;
  double y_target = 0;
  double z_target = 0;

  // Constants - Maybe move these...?
  unsigned int pwm_min = 50;
  unsigned int pwm_max = 255;
  double lpf_mult = 0.3;  // Sensor is 100hz, so desired lpf frequency / 100 = lpf_mult
  double x_kp = 0.30, x_ki = 0.0, x_kd = 0.0;
  double y_kp = 0.30, y_ki = 0.0, y_kd = 0.0;
  double z_kp = 0.02, z_ki = 0.0, z_kd = 0.0;

  // PID Controllers
  PID x_controller = PID(&x_position, &x_power, &x_target, x_kp, x_ki, x_kd, DIRECT);
  PID y_controller = PID(&y_position, &y_power, &y_target, y_kp, y_ki, y_kd, DIRECT);
  PID z_controller = PID(&z_position, &z_power, &z_target, z_kp, z_ki, z_kd, DIRECT);
};
