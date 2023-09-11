class PIDController {
private:
    double Kp_, Ki_, Kd_;
    double previous_error_;
    double integral_;

public:
    PIDController(double Kp, double Ki, double Kd)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), previous_error_(0), integral_(0) {}

    double compute(double setpoint, double measured_value) {
        double error = setpoint - measured_value;
        integral_ += error;
        double derivative = error - previous_error_;
        double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        previous_error_ = error;
        return output;
    }
};
