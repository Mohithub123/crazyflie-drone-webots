#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

#define TIME_STEP 32

int main() {

  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

  WbDeviceTag m1 = wb_robot_get_device("m1_motor");
  WbDeviceTag m2 = wb_robot_get_device("m2_motor");
  WbDeviceTag m3 = wb_robot_get_device("m3_motor");
  WbDeviceTag m4 = wb_robot_get_device("m4_motor");

  wb_motor_set_position(m1, INFINITY);
  wb_motor_set_position(m2, INFINITY);
  wb_motor_set_position(m3, INFINITY);
  wb_motor_set_position(m4, INFINITY);

  double thrust = 68.5;

  while (wb_robot_step(TIME_STEP) != -1) {

    int key = wb_keyboard_get_key();

    double m1_speed = thrust;
    double m2_speed = thrust;
    double m3_speed = thrust;
    double m4_speed = thrust;

    if (key == 'W') {
      m1_speed += 2;
      m2_speed += 2;
    }

    if (key == 'S') {
      m3_speed += 2;
      m4_speed += 2;
    }

    if (key == 'A') {
      m1_speed += 2;
      m3_speed += 2;
    }

    if (key == 'D') {
      m2_speed += 2;
      m4_speed += 2;
    }

    wb_motor_set_velocity(m1, m1_speed);
    wb_motor_set_velocity(m2, -m2_speed);
    wb_motor_set_velocity(m3, m3_speed);
    wb_motor_set_velocity(m4, -m4_speed);
  }

  wb_robot_cleanup();
  return 0;
}