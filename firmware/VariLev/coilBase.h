#include <PID_v2.h>
#include <Arduino.h>
#include <Tlv493d.h>

#define SENSORFREQ 300 // 300 1000
#define CYCLEFREQ 300 // 300 1000

static bool DEBUGPRINT = true;

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
  int cal_powers[16] = {-255, -235, -215, -195, -175, -155, -135, -115, 115, 135, 155, 175, 195, 215, 235, 255};
  double xcal[16], ycal[16], zcal[16];
  bool calibrated = false;
  double xdist = 0, ydist = 0, zdist = 0;
  double lpf_cal = 100.0 / SENSORFREQ;
  
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
  double input_correction(double pos);
  void tip_correction(double &x, double &y);
  void update_tip_parameters(double rise, double powmult, double maxpow);
  
  
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

  // magnet tip compensation
  double risetime_s = 0.040;
  double deflection_mult = 2.5;
  double deflection_max = 2.5;
  double xDeflect = 0.0;
  double yDeflect = 0.0;
  double maxTipDelta = deflection_max / (risetime_s * SENSORFREQ);

  // Constants
  double lpf_mult = 10.0 / SENSORFREQ; // float is cutoff freq
  double x_kp = 0.0, x_ki = 0.0, x_kd = 0.02; // x_kp = 0.15, x_ki = 0.005, x_kd = 0.001;
  double y_kp = 0.0, y_ki = 0.0, y_kd = 0.02; // y_kp = 0.15, y_ki = 0.005, y_kd = 0.001;
  double z_kp = 0.02, z_ki = 0.0, z_kd = 0.0;

  // 0.15, 0.005, 0.001, 0.050, 3.0, 3.0

  // PID Controllers
  PID_v2 x_controller = PID_v2(x_kp, x_ki, x_kd, PID::Reverse);
  PID_v2 y_controller = PID_v2(y_kp, y_ki, y_kd, PID::Reverse);
  PID_v2 z_controller = PID_v2(z_kp, z_ki, z_kd, PID::Direct);
};
