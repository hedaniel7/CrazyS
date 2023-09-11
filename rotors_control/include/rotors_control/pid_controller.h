#pragma once

class PIDController {
private:
    double Kp_, Ki_, Kd_;
    double previous_error_;
    double integral_;

public:
    // Constructor
    PIDController(double Kp, double Ki, double Kd);

    // Compute the PID output based on the setpoint and measured value
    double compute(double setpoint, double measured_value);
};
