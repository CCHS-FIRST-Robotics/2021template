package frc.robot.ai.subroutines;

import frc.robot.helper.PID;
import frc.robot.Constants;
import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

public class TurnToPoint {
    double target_pos[];
    PID turn_pid;
    Command main_command;
    double end_time = 3;
    double start_time_sec = 0;

    public TurnToPoint(double target_x, double target_y) {
        this.target_pos[0] = target_x;
        this.target_pos[1] = target_y;
        this.turn_pid = new PID(0.5, 0, 0);
        this.main_command = new Command(0, 0);
    }

    public boolean exit(MainState main_state) {
        double remaining = Math.abs(getTheta(main_state));
        if (remaining < Constants.ACCEPTABLE_ANGLE_ERROR) {
            return true;
        }
        double ctime = (double) System.currentTimeMillis() / 1000;
        if (ctime - this.start_time_sec > this.end_time) {
            return true;
        }
        return false;
    }

    public void initExit(MainState main_state) {
        this.start_time_sec = (double) System.currentTimeMillis() / 1000; // Start "timer" here
        this.end_time = Math.abs(getTheta(main_state)) * Constants.TURN_RATE;
    }

    public double getTheta(MainState main_state) {
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.target_pos[0] - pos[0], this.target_pos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle(unit_h_vec, point_vec);

        return pwr_cmd;
    }

    public Command update(MainState main_state) {
        this.main_command.diffDrive(0, getTheta(main_state));
        return this.main_command;
    }
}
