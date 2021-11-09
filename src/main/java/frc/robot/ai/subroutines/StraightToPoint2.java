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

    public StraightToPoint2(double target_x, double target_y) {
        this.forward_pid = new PID(0.2, 0.0, 0); // must be k_i = 0
        this.target[0] = target_x;
        this.target[1] = target_y;
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
        return ExitMethods.fusionExit(vel_mag, t_dist);
    }

    public void initExit(MainState main_state) {
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        this.start_time_sec = System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = ExitMethods.targetTime(t_dist);
        this.max_dist = t_dist;
        this.score_coeff = ExitMethods.initScoreCoeff(t_dist);
    }

    double rScalar() {
        double c_time = (double) System.currentTimeMillis() / 1000;
        double turn_max_t = Math.min(Constants.TURN_TIME_MAX, Constants.TURN_TIME_FAC * this.end_time);
        double prop_comp = (c_time - this.start_time_sec) * (1 - Constants.MIN_R_FAC)
                / (Constants.TURN_TIME_FAC * this.end_time);
        double output = Math.min(prop_comp + Constants.MIN_R_FAC, 1);
        return output;
    }

    public Command update(MainState main_state) {
        double[] h_hat = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double[] current_pos = main_state.getPosVal();
        double[] ortho_vec = { h_hat[1], -1 * h_hat[0] };
        double[] y_prime = { this.target[0] - current_pos[0], this.target[1] - current_pos[1] };
        double t_dist = SimpleMat.mag(y_prime);
        double r = t_dist / (2 * SimpleMat.dot(y_prime, ortho_vec));
        r = r * rScalar();
        double[] prop_command = { r + Constants.ROBOT_WIDTH / 2, r - Constants.ROBOT_WIDTH / 2 };
        double max_prop_mag = Math.max(Math.abs(prop_command[0]), Math.abs(prop_command[1]));
        if (max_prop_mag == -1 * Math.min(prop_command[0], prop_command[1])) {
            prop_command[0] = prop_command[0] * -1;
            prop_command[1] = prop_command[1] * -1;
        }
        double pwr = Math.max(Math.min(this.forward_pid.update(t_dist), 1), 0.05);
        double[] t_cmd = SimpleMat.scaleVec(prop_command, pwr / (max_prop_mag + 0.0001));

        Command output = new Command(t_cmd[0], t_cmd[1]);

        return output;
    }
}
