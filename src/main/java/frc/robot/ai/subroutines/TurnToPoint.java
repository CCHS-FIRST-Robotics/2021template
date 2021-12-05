package frc.robot.ai.subroutines;

import frc.robot.helper.PID;
import frc.robot.Constants;
import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

/**
 * Method for generating commands to point the robot on the spot to the given
 * target point
 * 
 * @author Ludwig Tay
 */
public class TurnToPoint {
    double target_pos[] = { 0, 0 };
    PID turn_pid;
    Command main_command;
    double end_time = 3;
    double start_time_sec = 0;

    /**
     * Constructor for TurnToPoint, takes the target coordinate in meters
     * 
     * @param target_x
     * @param target_y
     */
    public TurnToPoint(double target_x, double target_y) {
        this.target_pos[0] = target_x;
        this.target_pos[1] = target_y;
        this.turn_pid = new PID(Constants.TURN_TUNING[0], Constants.TURN_TUNING[1], Constants.TURN_TUNING[2]);
        this.main_command = new Command(0, 0);
    }

    /**
     * Method that determines when to exit through various methods: remaining angle
     * to travel < threshold time > threshold
     * 
     * @param main_state main_state of the robot
     * @return boolean of whether to exit (true for exit)
     */
    public boolean exit(MainState main_state) {
        double remaining = Math.abs(getTheta(main_state));
        if (remaining < Constants.ACCEPTABLE_ANGLE_ERROR) {
            return true;
        }
        double ctime = (double) System.currentTimeMillis() / 1000;
        if ((ctime - this.start_time_sec) > this.end_time) {
            return true;
        }
        return false;
    }

    /**
     * Initialize values needed for exit conditions such as the allotted time.
     * 
     * @param main_state main_state of robot
     */
    public void initExit(MainState main_state) {
        double max_ang_vel = (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS
                * Constants.INIT_L_WHL_TRAC / (Constants.ROBOT_WIDTH / 2);
        this.start_time_sec = (double) System.currentTimeMillis() / 1000; // Start "timer" here
        // this.end_time = Math.abs(getTheta(main_state)) * Constants.TURN_LEEWAY /
        // max_ang_vel;
        this.end_time = 3;
    }

    /**
     * Computes the angle difference between current heading and the target heading.
     * Positive and negative for direction
     * 
     * @param main_state main state of the robot
     * @return double of the remaining angle in radians (positive for ccw turn,
     *         negative for cw turn)
     */
    public double getTheta(MainState main_state) {
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.target_pos[0] - pos[0], this.target_pos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);
        double pwr_cmd = SimpleMat.vecsAngle2(unit_h_vec, point_vec);

        return pwr_cmd;
    }

    /**
     * Generates hardware command needed to turn the robot to point to target point.
     * 
     * @param main_state main state of the robot.
     * @return hardware command of the robot.
     */
    public Command update(MainState main_state) {
        this.main_command.diffDrive(0, this.turn_pid.update(getTheta(main_state)));
        return this.main_command;
    }
}
