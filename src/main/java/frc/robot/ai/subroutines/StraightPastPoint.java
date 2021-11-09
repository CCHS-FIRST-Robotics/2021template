package frc.robot.ai.subroutines;

import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.helper.PID;
import frc.robot.helper.SimpleMat;
import frc.robot.Constants;
import frc.robot.ai.subroutines.exit_methods.ExitMethods;

public class StraightPastPoint {
    double[] target = { 0, 0 };
    double start_time_sec;
    double end_time;
    double max_dist;
    double start_r;
    double pwr;
    double[] y2 = { 0, 0 };
    double[] line_norm = { 0, 1 };

    public StraightPastPoint(double target_x, double target_y, double pwr) {
        this.pwr = pwr;
        this.target[0] = target_x;
        this.target[1] = target_y;
    }

    public boolean exit(MainState main_state) {
        double ctime = (double) System.currentTimeMillis() / 1000;
        if ((ctime - this.start_time_sec) > this.end_time) {
            return true;
        }
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        if (t_dist < Constants.ACCEPTABLE_DIST_ERROR) {
            return true;
        }
        double[] diff_vec = { main_state.getPosVal()[0] - this.y2[0], main_state.getPosVal()[1] - this.y2[1] };
        if (SimpleMat.dot(diff_vec, this.line_norm) > 0) {
            return true;
        }
        return false;
    }

    public void initExit(MainState main_state) {
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        this.start_time_sec = System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = ExitMethods.targetTime(t_dist);
        this.max_dist = t_dist;

        double[] current_pos = main_state.getPosVal();
        double[] y_prime = { this.target[0] - current_pos[0], this.target[1] - current_pos[1] };
        this.line_norm = SimpleMat.scaleVec(y_prime, (t_dist - Constants.ACCEPTABLE_DIST_ERROR) / t_dist);
        this.y2[0] = current_pos[0] + this.line_norm[0];
        this.y2[1] = current_pos[1] + this.line_norm[1];
    }

    double rScalar() {
        double c_time = (double) System.currentTimeMillis() / 1000;
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

        double[] t_cmd = SimpleMat.scaleVec(prop_command, this.pwr / (max_prop_mag + 0.0001));

        Command output = new Command(t_cmd[0], t_cmd[1]);

        return output;
    }
}