package frc.robot.ai.subroutines;

import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.helper.PID;
import frc.robot.helper.SimpleMat;
import frc.robot.Constants;
import frc.robot.ai.subroutines.exit_methods.ExitMethods;

public class StraightToPoint2 {
    PID forward_pid;
    double[] target = { 0, 0 };
    double start_time_sec;
    double end_time;
    double max_dist;
    double start_r;
    double score_coeff;
    double previous_pwr;
    double previous_ad;
    double[] line_norm = { 0, 1 };

    public StraightToPoint2(double target_x, double target_y) {
        this.forward_pid = new PID(0.2, 0.0, 0); // must be k_i = 0
        this.target[0] = target_x;
        this.target[1] = target_y;
        this.previous_pwr = 0;
    }

    public boolean exit(MainState main_state) {
        double ctime = (double) System.currentTimeMillis() / 1000;
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        double score = (ctime - this.start_time_sec) + score_coeff / t_dist;
        if (score > this.end_time) {
            return true;
        }
        if (t_dist < Constants.ACCEPTABLE_DIST_ERROR) {
            return true;
        }
        double vel_mag = SimpleMat.mag(main_state.getVelVal());
        if (ExitMethods.fusionExit(vel_mag, t_dist)) {
            return true;
        }
        if (ExitMethods.thetaExit(main_state, this.target) && t_dist < Constants.ACCEPTABLE_DIST_SOFT) {
            return true;
        }
        return false;
    }

    public void initExit(MainState main_state) {
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        this.start_time_sec = System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = ExitMethods.targetTime(t_dist);
        this.max_dist = t_dist;
        this.score_coeff = ExitMethods.initScoreCoeff(t_dist);
    }

    double arcDistance(MainState state, double r) {
        double[] delta = { this.target[0] - state.getPosVal()[0], this.target[1] - state.getPosVal()[1] };
        double[] heading = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double smaller_angle = Math.acos(SimpleMat.dot(delta, heading) / SimpleMat.mag(delta));
        double dist_mag = 2 * smaller_angle * r;

        // determining r origin point.
        double[] left_h = SimpleMat.projectHeading(state.getHeadingVal() + Math.PI / 2, r);
        double[] right_h = SimpleMat.projectHeading(state.getHeadingVal() - Math.PI / 2, r);
        left_h = SimpleMat.add(left_h, state.getPosVal());
        right_h = SimpleMat.add(right_h, state.getPosVal());

        double left_c = SimpleMat.mag(SimpleMat.subtract(left_h, this.target));
        double right_c = SimpleMat.mag(SimpleMat.subtract(right_h, this.target));

        double[] r_o = { 0, 0 };
        if (left_c < right_c) {
            r_o = left_h;
        } else {
            r_o = right_h;
        }

        double[] r_o_t = SimpleMat.rot2d(SimpleMat.subtract(this.target, r_o), Math.PI / 2);
        double[] r_o_p = SimpleMat.subtract(state.getPosVal(), r_o);

        if (SimpleMat.dot(r_o_t, r_o_p) < 0) {
            return dist_mag * -1;
        } else {
            return dist_mag * 1;
        }
    }

    double pwrController(MainState state, double dist_mag) {
        double vel = (dist_mag - this.previous_ad) / Constants.MAIN_DT;
        double max_vel = Constants.MOTOR_MAX_RPM * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC;
        double max_acc = (2 * Constants.MOTOR_MAX_TORQUE / Constants.WHEEL_RADIUS) / Constants.ROBOT_MASS;
        this.previous_ad = dist_mag;
        double drift_dist = -1 * max_vel * max_vel / (Constants.INIT_FRICTION * Constants.GRAV_ACC);
        if (drift_dist < dist_mag) {
            this.previous_pwr = 1;
            return 1;
        }
        double discriminant = 4 * vel * vel + 6 * max_acc * this.previous_pwr * dist_mag;
        if (discriminant < 0) {
            double des_acc = -1 * (2 / 3) * (vel * vel) / dist_mag;
            double pwr = des_acc / max_acc;
            this.previous_pwr = pwr;
            return pwr;
        }
        double n = -2 * vel + Math.pow(discriminant, 0.5) * (1 / (max_acc * this.previous_pwr));
        double slope = -2 * (vel + (max_acc * this.previous_pwr) * n) / (n * n);
        double pwr = ((max_acc * this.previous_pwr) + slope * Constants.MAIN_DT) / max_acc;
        this.previous_pwr = pwr;
        return pwr;
    }

    double rScalar() {
        double c_time = (double) System.currentTimeMillis() / 1000;
        double turn_max_t = Math.min(Constants.TURN_TIME_MAX, Constants.TURN_TIME_FAC * this.end_time);
        double prop_comp = (c_time - this.start_time_sec) * (1 - Constants.MIN_R_FAC)
                / (Constants.TURN_TIME_FAC * this.end_time);
        double output = Math.min(prop_comp + Constants.MIN_R_FAC, 0.9);
        return output;
    }

    public Command update(MainState main_state) {
        double[] h_hat = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double[] current_pos = main_state.getPosVal();
        double[] ortho_vec = { h_hat[1], -1 * h_hat[0] };
        double[] y_prime = { this.target[0] - current_pos[0], this.target[1] - current_pos[1] };
        double t_dist = SimpleMat.mag(y_prime);
        double r = t_dist / (2 * SimpleMat.dot(y_prime, ortho_vec));
        double arc_dist = arcDistance(main_state, r);
        r = r * rScalar();
        double[] prop_command = { r + Constants.ROBOT_WIDTH / 2, r - Constants.ROBOT_WIDTH / 2 };
        double max_prop_mag = Math.max(Math.abs(prop_command[0]), Math.abs(prop_command[1]));
        if (max_prop_mag == -1 * Math.min(prop_command[0], prop_command[1])) {
            prop_command[0] = prop_command[0] * -1;
            prop_command[1] = prop_command[1] * -1;
        }
        double pwr = Math.max(Math.min(pwrController(main_state, arc_dist), 1), -1);
        double[] t_cmd = SimpleMat.scaleVec(prop_command, pwr / (max_prop_mag + 0.0001));

        Command output = new Command(t_cmd[0], t_cmd[1]);

        return output;
    }
}
