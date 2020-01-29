#include "app_maverick_gen.h"

double map(double input, double in_min, double in_max, double out_min, double out_max) {
  return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double calculate_pid(steering_pid_values pid, double current_pos, double set_pos) {
    static double integral = 0;
    static double last_error = 0;

    // Calculate Error
    double error = set_pos - current_pos;

    if (error < STEERING_DEADBAND && error > -STEERING_DEADBAND) error = 0;

    // Calculate Integral
    integral += error;
    // Calculate Derivative
    double derivative = error - last_error;

    // Calculate PID output
    double out = pid.kp * error + pid.ki * integral + pid.kd * derivative;
    // Ensure it doesn't go outside duty range
    if (out > MAX_STEERING_DUTY) out = MAX_STEERING_DUTY;
    else if (out < -MAX_STEERING_DUTY) out = -MAX_STEERING_DUTY;
    else if (out < MIN_STEERING_DUTY && out > -MIN_STEERING_DUTY) out = 0;

    last_error = error;
    return out;
}