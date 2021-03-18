#include <PID_v1.h>
#include <Arduino.h>

struct VariLev
{
  VariLev(unsigned int coils[4]);
  // Coil initialization should be in order x-, x+, y-, y+
  
  public:
  
  bool update_current_mags(int x, int y, int z);
  bool set_mags_target(int x, int y, int z);
  
  bool get_mags_target(int &x, int &y, int &z);
  
  bool enable_controllers();
  bool disable_controllers();
  
  
  private:
  
  bool compute_pid();
  bool update_outputs();
  bool calculate_pwm(double percent, unsigned int &pwm_neg, unsigned int &pwm_pos);
  
  unsigned int coil_x_n = 0;
  unsigned int coil_x_p = 0;
  unsigned int coil_y_n = 0;
  unsigned int coil_y_p = 0;
  
  double x_position = 0;
  double y_position = 0;
  double z_position = 0;
  
  double x_power = 0.5;
  double y_power = 0.5;
  double z_power = 0.5;
  
  double x_target = 0;
  double y_target = 0;
  double z_target = 0;


  // Constants - Maybe move these...?
  unsigned int pwm_min = 70;
  unsigned int pwm_max = 255;
  double x_kp = 1.0, x_ki = 0.0, x_kd = 0.0;
  double y_kp = 1.0, y_ki = 0.0, y_kd = 0.0;
  double z_kp = 1.0, z_ki = 0.0, z_kd = 0.0;

  // PID Controllers
  PID x_controller = PID(&x_position, &x_power, &x_target, x_kp, x_ki, x_kd, DIRECT);
  PID y_controller = PID(&y_position, &y_power, &y_target, y_kp, y_ki, y_kd, DIRECT);
  PID z_controller = PID(&z_position, &z_power, &z_target, z_kp, z_ki, z_kd, DIRECT);
};
