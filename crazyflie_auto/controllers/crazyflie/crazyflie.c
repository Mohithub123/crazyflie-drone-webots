#include <math.h>
#include <stdio.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define TARGET_ALTITUDE 0.5

#define TARGET_X 1.5
#define TARGET_Y 1.5

int main() {

  wb_robot_init();

  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag gyro = wb_robot_get_device("gyro");

  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);

  WbDeviceTag m1 = wb_robot_get_device("m1_motor");
  WbDeviceTag m2 = wb_robot_get_device("m2_motor");
  WbDeviceTag m3 = wb_robot_get_device("m3_motor");
  WbDeviceTag m4 = wb_robot_get_device("m4_motor");

  wb_motor_set_position(m1, INFINITY);
  wb_motor_set_position(m2, INFINITY);
  wb_motor_set_position(m3, INFINITY);
  wb_motor_set_position(m4, INFINITY);

  double base_speed = 68;

  while (wb_robot_step(TIME_STEP) != -1) {

    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double roll = rpy[0];
    double pitch = rpy[1];

    const double *pos = wb_gps_get_values(gps);

    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    /* altitude control */

    double altitude_error = TARGET_ALTITUDE - z;
    double vertical = altitude_error * 10;

    /* navigation control */

    double dx = TARGET_X - x;
    double dy = TARGET_Y - y;

    double forward = dx * 5;
    double sideways = dy * 5;

    /* stabilization */

    double roll_input = roll * 50;
    double pitch_input = pitch * 30;

    double m1_speed = base_speed + vertical - roll_input + pitch_input + sideways;
    double m2_speed = base_speed + vertical - roll_input - pitch_input - sideways;
    double m3_speed = base_speed + vertical + roll_input - pitch_input + sideways;
    double m4_speed = base_speed + vertical + roll_input + pitch_input - sideways;

    wb_motor_set_velocity(m1, m1_speed);
    wb_motor_set_velocity(m2, -m2_speed);
    wb_motor_set_velocity(m3, m3_speed);
    wb_motor_set_velocity(m4, -m4_speed);
  }

  wb_robot_cleanup();
  return 0;
}