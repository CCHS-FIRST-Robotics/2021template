package frc.robot.ai.subroutines;

import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.helper.PID;
import frc.robot.helper.SimpleMat;
import frc.robot.Constants;
import frc.robot.ai.subroutines.exit_methods.ExitMethods;

/**
 * StraightToPoint2 --- class that gives commands to drive from current location
 * to output point along with exit condition to tell autonomous when it is done
 * travelling.
 * 
 * @author Ludwig Tay
 */
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

    /**
     * Constructor, takes the x and y coordinate of the target in meters.
     * 
     * @param target_x
     * @param target_y
     */
    public StraightToPoint2(double target_x, double target_y) {
        this.forward_pid = new PID(Constants.FORWARD_TUNING[0], Constants.FORWARD_TUNING[1],
                Constants.FORWARD_TUNING[2]); // must be k_i = 0
        this.target[0] = target_x;
        this.target[1] = target_y;
        this.previous_pwr = 0;
    }

    /**
     * Determines whether travel is complete through a variety of conditions: time +
     * dist score > exit time (timer based exit) distance to target < acceptable
     * distance distance and timer exit Angle of heading to target point > constant
     * angle
     * 
     * @param main_state (current robot state)
     * @return true/false on whether to exit (true for exit)
     */
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

    /**
     * Initialized values needed in exit conditions such as alloted time and initial
     * distance
     * 
     * @param main_state
     * @return none
     */
    public void initExit(MainState main_state) {
        double t_dist = SimpleMat.vectorDistance(this.target, main_state.getPosVal());
        this.start_time_sec = System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = ExitMethods.targetTime(t_dist);
        this.max_dist = t_dist;
        this.score_coeff = ExitMethods.initScoreCoeff(t_dist);
        this.previous_ad = t_dist;
    }

    /**
     * Calculates the remaining distance needed to travel to target in the constant
     * radius arc along with direction (Positive for forward, Negative for reverse)
     * 
     * @param state main_state of robot
     * @param r     radius of the arc needed to travel from current pos to target
     * @return double of arc distance in meters, can be positive or negative
     */
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

    /**
     * Decreases radius by a scalar factor to promote overcorrection in early parts
     * of travel to avoid huge sweeping arcs
     * 
     * @return double scalar to multiply radius from [min r frac, 1]
     */
    double rScalar() {
        double c_time = (double) System.currentTimeMillis() / 1000;
        double turn_max_t = Math.min(Constants.TURN_TIME_MAX, Constants.TURN_TIME_FAC * this.end_time);
        double prop_comp = (c_time - this.start_time_sec) * (1 - Constants.MIN_R_FAC)
                / (Constants.TURN_TIME_FAC * this.end_time);
        double output = Math.min(prop_comp + Constants.MIN_R_FAC, 1);
        return output;
    }
    
    /**
     * Checks whether an acceleration/deceleration to and from max velocity will 
     * within the planned distance
     * 
     * @param init_vel current velocity of the robot
     * @param distance proposed distanced to travel
     * @return
     */
    boolean maxVelocityCheck(double init_vel, double distance) {
        double max_vel = (2.0 * Math.PI * Constants.WHEEL_RADIUS) * (Constants.MOTOR_MAX_RPM / 60.0);
        double max_accel = (Constants.MOTOR_MAX_TORQUE / Constants.WHEEL_RADIUS) / Constants.ROBOT_MASS;
        double max_decel = -max_accel;
        
        // Trapezoid area
        double max_accel_diff = max_vel - init_vel; 
        double accel_time = Math.sqrt((max_accel * max_accel) - (max_accel_diff * max_accel_diff));
        double accel_dist = ((init_vel + max_vel) / 2) * accel_time;

        // Triangle area
        double decel_time = Math.sqrt((max_decel * max_decel) - (max_vel * max_vel));
        double decel_dist = (max_vel * decel_time) / 2;

        if (accel_dist + decel_dist < distance) {
            return true;
        }

        return false;
    }

    /**
     * Computes command to give to hardware objects to travel from current location
     * to the point.
     * 
     * @param main_state main_state of the robot.
     * @return Command, command to give to the hardware.
     */
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
        // double pwr = Math.max(Math.min(pwrController(main_state, arc_dist), 1), -1);
        double pwr = Math.max(Math.min(this.forward_pid.update(arc_dist), 1), -1);
        double[] t_cmd = SimpleMat.scaleVec(prop_command, pwr / (max_prop_mag + 0.0001));

        Command output = new Command(t_cmd[0], t_cmd[1]);

        return output;
    }
}
