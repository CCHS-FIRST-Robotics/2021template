package frc.robot.helper;

import frc.robot.Constants;

public class PID {
    double integral = 0;
    double previous;
    double previous_time;

    double k_p = 0.1;
    double k_i = 0.01;
    double k_d = 0.01;
    double decay = Math.exp(Math.log(0.5) / (Constants.INTERGRAL_HALFLIFE_T / Constants.MAIN_DT));

    public PID(double s_p, double s_i, double s_d) {
        this.k_p = s_p;
        this.k_i = s_i;
        this.k_d = s_d;

        this.decay = Math.exp(Math.log(0.5) / (Constants.INTERGRAL_HALFLIFE_T / Constants.MAIN_DT));
        reset();
    }

    public void setGain(double new_p) {
        this.k_p = new_p;
    }

    public void reset() {
        this.integral = 0;
        this.previous_time = (double) System.currentTimeMillis() / 1000;
    }

    public double update(double delta) {
        double current_time = (double) System.currentTimeMillis() / 1000;
        double dt = current_time - this.previous_time;
        if (dt == 0) {
            dt = 0.0001;
        }
        double deriv = (delta - this.previous) / dt;

        this.integral = this.integral * this.decay + delta * dt;

        double response = k_p * delta + k_i * this.integral + k_d * deriv;

        this.previous = delta;
        this.previous_time = current_time;
        return response;
    }
}