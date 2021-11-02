package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.ai.subroutines.exit_methods.ExitMethods;
import frc.robot.commands.Command;
import frc.robot.helper.SimpleMat;
import frc.robot.helper.PID;
import java.lang.Math;

public class StraightToPoint {
    double[] final_pos = { 0, 0 };
    double start_time_sec = 0;
    double travelDistance = 0;
    double end_time = 3;
    int updates = 0;
    PID turn_pid;
    PID forward_pid;
    Command main_command;

    public StraightToPoint(double target_x, double target_y) {
        this.final_pos[0] = target_x;
        this.final_pos[1] = target_y;
        this.turn_pid = new PID(0.5, 0, 0);
        this.forward_pid = new PID(0.2, 0, 0);
        this.main_command = new Command(0, 0);
    }

    public boolean exit(MainState main_state) {
        double ctime = (double) System.currentTimeMillis() / 1000;
        if ((ctime - this.start_time_sec) > this.start_time_sec) {
            return true;
        }
        double t_dist = SimpleMat.vectorDistance(this.final_pos, main_state.getPosVal());
        if (t_dist < Constants.ACCEPTABLE_DIST_ERROR) {
            return true;
        }
        return false;
    }

    public void initExit(MainState main_state) {
        double t_dist = SimpleMat.vectorDistance(this.final_pos, main_state.getPosVal());
        this.start_time_sec = System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = ExitMethods.targetTime(t_dist);
    }

    public Command update(MainState main_state) {
        // Compute the forward back vector factor
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.final_pos[0] - pos[0], this.final_pos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);

        double fwd_mag_fac = SimpleMat.dot(point_vec, unit_h_vec);

        double t_dist = SimpleMat.vectorDistance(this.final_pos, pos);

        boolean orient = true;
        if (t_dist < Constants.NO_ORIENT_DIST) {
            orient = false;
        }
        double turn_angle = turnAngle(unit_h_vec, point_vec, orient);

        double fwd_response = forward_pid.update(t_dist * fwd_mag_fac);
        double turn_response = turn_pid.update(turn_angle);

        this.main_command.diffDrive(fwd_response, turn_response);

        if (updates == 0) {
            initExit(main_state);
        }

        this.updates++;
        return this.main_command;
    }

    public double turnAngle(double[] h_vec, double[] point_vec, boolean orient) {
        double orient_angle = SimpleMat.vecsAngle(h_vec, point_vec);
        if (orient) {
            return orient_angle;
        }
        double[] opposing_h_vec = { h_vec[0] * -1, h_vec[1] * -1 };
        double opposing_angle = SimpleMat.vecsAngle(opposing_h_vec, point_vec);
        if (Math.abs(opposing_angle) < Math.abs(orient_angle)) {
            return opposing_angle;
        } else {
            return orient_angle;
        }
    }
}