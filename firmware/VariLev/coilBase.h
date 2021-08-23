#include <PID_v2.h>
#include <Arduino.h>
#include <Tlv493d.h>

#define CYCLEFREQ 1000 // 250 1000

struct Coil
{
  Coil() {};
  Coil(unsigned int dirPin0, unsigned int enPin0, unsigned int dirPin1, unsigned int enPin1, bool invert0, bool invert1);

  public:
  bool run_cal(Tlv493d &MagSensor, float caltime_s);
  bool set_pins(unsigned int dirPin0, unsigned int enPin0, unsigned int dirPin1, unsigned int enPin1);
  bool set_power(int power);
  bool get_distortion(double &x, double &y, double &z);

  private:
  unsigned int dir_pin0, dir_pin1, en_pin0, en_pin1;
  int flip0 = false, flip1 = false;
  int coil_power = 0;
  int cal_powers[17] = {-255, -225, -195, -165, -135, -105, -65, 30, 0, 30, 65, 105, 135, 165, 195, 225, 255};
  double xcal[17], ycal[17], zcal[17];
  bool calibrated = false;
  
};

struct VariLev
{
  VariLev(Coil coils[2]);
  
  public:
  
  bool update_current_mags(double x, double y, double z);
  bool set_mags_target(double x, double y, double z);
  bool get_mags_target(double &x, double &y, double &z);
  
  bool enable_controllers();
  bool disable_controllers();

  void update_xy_tuning(double kp, double ki, double kv);
  
  
  private:
  
  bool update_outputs();
  bool calculate_pwm(double percent, unsigned int &pwm_neg, unsigned int &pwm_pos);
  bool z_mag_to_mm(double zmag, double &zdist);
  
  Coil coil_x, coil_y;
  
  double x_position = 0;
  double y_position = 0;
  double z_position = 0;
  
  double x_power = 0.0;
  double y_power = 0.0;
  double z_power = 0.5;

  // Constants - Maybe move these...?
  unsigned int pwm_min = 50;
  unsigned int pwm_max = 255;
  double lpf_mult = 400.0 / CYCLEFREQ;  // Sensor is 100hz, so desired lpf frequency / 100 = lpf_mult
  double out_lpf_mult = 1.0; // 200.0 / CYCLEFREQ;
  double x_kp = 0.1, x_ki = 0.0, x_kd = 0.0;
  double y_kp = 0.1, y_ki = 0.0, y_kd = 0.0;
  double z_kp = 0.02, z_ki = 0.0, z_kd = 0.0;

  // PID Controllers
  PID_v2 x_controller = PID_v2(x_kp, x_ki, x_kd, PID::Reverse);
  PID_v2 y_controller = PID_v2(y_kp, y_ki, y_kd, PID::Reverse);
  PID_v2 z_controller = PID_v2(z_kp, z_ki, z_kd, PID::Direct);
};
