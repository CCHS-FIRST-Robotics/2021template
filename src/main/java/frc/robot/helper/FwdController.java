package frc.robot.helper;

import frc.robot.Constants;
import frc.robot.state.MainState;

public class FwdController {
    double v_max = 0.5 * Constants.MOTOR_MAX_RPM * 2 * Math.PI * Constants.WHEEL_RADIUS / 60;
    double a_max = 0.5 * (2 * Constants.MOTOR_MAX_TORQUE / Constants.WHEEL_RADIUS) / Constants.ROBOT_MASS;
    double jerk = 3;

    double target_v = 0;

    PID whl_c;

    public FwdController() {
        this.target_v = 0;
        this.whl_c = new PID(Constants.C_BASE_GAIN, Constants.C_BASE_GAIN * 0.1, Constants.C_BASE_GAIN * 0.001);
    }

    double getAcc(double clean_x, double clean_v) {
        // Check if its too far
        if (Math.pow(clean_x / this.a_max, 0.5) < clean_x / this.v_max) {
            return this.a_max;
        }
        // Compute lin straight down
        double lsd = Math.pow(clean_v, 2) / (2 * clean_x);
        // Newtons method with convergence check
        double prev = a_max;
        double deriv = 0;
        double a = a_max;
        for (int i = 0; i < 10; i++) {
            if (a < 0) {
                return lsd;
            }
            double fx = (2 * a * a / this.jerk) - 2 * Math.pow(a * clean_x, 0.5) + clean_v;
            double fxp = (4 * a / this.jerk) + clean_v - Math.pow(clean_x / a, 0.5);
            a = a - fx / fxp;
            deriv = a - prev;
            prev = a;
            if (i > 7 && deriv > 0.2) {
                return lsd;
            }
        }
        return a;
    }

    public double update(MainState state, double x) {
        double current_v = (state.getLWhlRadssVal() * 0.5 + state.getRWhlRadssVal() * 0.5) * Constants.WHEEL_RADIUS;
        double dir;
        if (x > 0) {
            dir = 1;
        } else {
            dir = -1;
        }
        double acc = getAcc(x * dir, current_v * dir) * dir;
        double radpss = acc / Constants.WHEEL_RADIUS;
        target_v = target_v + radpss * Constants.MAIN_DT;
        double delta = target_v - (state.getLWhlRadssVal() * 0.5 + state.getRWhlRadssVal() * 0.5);
        double resp = this.whl_c.update(delta);
        resp = Math.min(1, Math.max(-1, resp));
        return resp;
    }
}
