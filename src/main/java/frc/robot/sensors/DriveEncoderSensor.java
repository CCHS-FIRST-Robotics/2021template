package frc.robot.sensors;

import frc.robot.sensors.BaseSensor;
import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.helper.SimpleMat;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import frc.robot.HardwareObjects;

/**
 * Class that manages drive encoder values and converts it to interpretable
 * values in main state by doing wheel odometry calculations
 * 
 * @author Ludwig Tay
 */
public class DriveEncoderSensor extends BaseSensor {

    double pos[] = { 0, 0 };
    double heading = Constants.INIT_HEADING;
    double ang_vel = Constants.INIT_ANG_VEL;

    double pos_var = 0;
    double heading_var = 0;

    double arc_angle = 0;
    double o_local_delta[] = { 0, 0 };

    public double log_l_radss = 0;
    public double log_r_radss = 0;
    public double log_h_pred = 0;
    public double log_hk_pred = 0;

    /**
     * Constructor for DriveEncoderSensor.
     * 
     * @param sync_time UNUSED time of system start for synchronization.
     */
    public DriveEncoderSensor(double sync_time) {
        this.LAG_TIME = 0.0; // No Lag
        this.SYNC_TIME = sync_time;
        this.last_updated = sync_time;
    }

    /**
     * Standard method to tell main loop whether sensor will give reasonable and
     * productive values. Always true (Potential for sensor health check implemented
     * here)
     * 
     * @return always true
     */
    public boolean shouldUse() {
        return true;
    }

    /**
     * Checking validity of sensor rad/s values
     * 
     * @param radss radians/second
     * @return true if inside limits
     */
    public boolean commonSense(double radss) {
        if (radss > 100) {
            return false;
        }
        if (radss < -100) {
            return false;
        }
        return true;
    }

    /**
     * Calculate the local displacement of the robot (does not take into account
     * current heading) from the motor encoder values. Doing wheel odometry
     * calculations.
     * 
     * @param l_radss left wheel radians/second
     * @param r_radss right wheel radians/second
     */
    public void localDisplacement(double l_radss, double r_radss, double dt) {
        double l = dt * l_radss * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC;
        double r = dt * r_radss * Constants.WHEEL_RADIUS * Constants.INIT_R_WHL_TRAC;

        this.arc_angle = (r - l) / Constants.ROBOT_WIDTH;

        if (r == l) {
            this.o_local_delta[0] = 0;
            this.o_local_delta[1] = r;
        } else {
            double travel_mag = Constants.ROBOT_WIDTH * (l + r) / (2 * (r - l));
            this.o_local_delta[0] = travel_mag * (Math.cos(arc_angle) - 1);
            this.o_local_delta[1] = travel_mag * (Math.sin(arc_angle));
        }
    }

    /**
     * Takes sensor values, calculate local displacement, transform with heading.
     * Generates a new position, heading, position, velocity, angular velocity that
     * is kalman updated into main state.
     * 
     * @param state    main state of robot.
     * @param hardware hardware object of the robot.
     */
    public void processValue(MainState state, HardwareObjects hardware) {

        double l_raw = hardware.LEFT_MOTOR1.getSelectedSensorVelocity(1);
        double r_raw = hardware.RIGHT_MOTOR1.getSelectedSensorVelocity(1);
        // Convert from RPM
        double l_radss = l_raw * 2 * Math.PI * -1 / (60 * 8.35);
        double r_radss = r_raw * 2 * Math.PI / (60 * 8.35);

        // Update wheel rpm state
        state.setLWhlRadss(l_radss, 0);
        state.setRWhlRadss(r_radss, 0);

        this.log_l_radss = l_radss;
        this.log_r_radss = r_radss;

        localDisplacement(l_radss, r_radss, Constants.MAIN_DT);

        double[] o_delta = { 0, 0 };
        o_delta = SimpleMat.rot2d(this.o_local_delta, this.heading);

        double[] pred_pos_m = { this.pos[0] + o_delta[0], this.pos[1] + o_delta[1] };

        double new_heading_m = this.heading + this.arc_angle;

        double[] pred_pos_o = SimpleMat.add(state.getWhlOdoPosVal(), o_delta);
        double new_heading_o = state.getWhlOdoHVal() + this.arc_angle;

        // compute variances
        double dist_coeff = (Math.abs(l_radss) + Math.abs(r_radss)) * Constants.MAIN_DT / (2 * Math.PI);

        double p_var = Constants.VAR_RAD_VAR * dist_coeff * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC;
        double diff_coeff = (Math.abs(l_radss - r_radss)) * Constants.MAIN_DT / (2 * Math.PI);
        double h_var = Constants.VAR_RAD_VAR * 4 * diff_coeff * Constants.MAIN_DT / Constants.ROBOT_WIDTH;

        // Kalman update first with m

        // Pos
        SmartDashboard.putNumber("Odo Pos Var", p_var);
        SmartDashboard.putNumber("Odo Pos Accum Var", state.getWhlOdoPosVar());
        SmartDashboard.putNumber("Current Pos Var", state.getPosVar());
        SmartDashboard.putNumber("Odo X Pos", state.getPosVal()[0]);
        SmartDashboard.putNumber("Odo Y Pos", state.getPosVal()[1]);
        double[] xpos = state.kalmanUpdate(state.getPosVal()[0], state.getPosVar(), pred_pos_m[0],
                this.pos_var + p_var);
        double[] ypos = state.kalmanUpdate(state.getPosVal()[1], state.getPosVar(), pred_pos_m[1],
                this.pos_var + p_var);
        double[] kpos = { xpos[0], ypos[0] };
        state.setPos(kpos, xpos[1]); // potential to set pred pos

        // Vel
        double[] xvel = state.kalmanUpdate(state.getVelVal()[0], state.getVelVar(), o_delta[0] / Constants.MAIN_DT,
                p_var / Constants.MAIN_DT);
        double[] yvel = state.kalmanUpdate(state.getVelVal()[1], state.getVelVar(), o_delta[1] / Constants.MAIN_DT,
                p_var / Constants.MAIN_DT);
        double[] kvel = { xvel[0], yvel[0] };
        state.setVel(kvel, xvel[1]);

        // Heading
        double[] kheading = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), new_heading_m,
                this.heading_var + h_var);
        this.log_h_pred = new_heading_o;
        this.log_hk_pred = kheading[0];
        state.setHeading(kheading[0], kheading[1]);

        // Ang Vel
        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(),
                this.arc_angle / Constants.MAIN_DT, h_var / Constants.MAIN_DT);
        state.setAngVel(kangvel[0], kangvel[1]);

        // Kalman Update with O

        // pos
        double[] xpos_o = state.kalmanUpdate(state.getPosVal()[0], state.getPosVar(), pred_pos_o[0],
                state.getWhlOdoPosVar() + p_var);
        double[] ypos_o = state.kalmanUpdate(state.getPosVal()[1], state.getPosVar(), pred_pos_o[1],
                state.getWhlOdoPosVar() + p_var);
        double[] kpos_o = { xpos_o[0], ypos_o[0] };
        state.setPos(kpos_o, xpos_o[1]); // potential to set pred pos
        // Heading
        double[] kheading_o = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), new_heading_o,
                state.getWhlOdoHVar() + h_var);
        state.setHeading(kheading_o[0], kheading_o[1]);

        state.setWhlOdoPos(pred_pos_o, state.getWhlOdoPosVar() + p_var);
        state.setWhlOdoH(new_heading_o, state.getWhlOdoHVar() + h_var);

        this.pos = state.getPosVal();
        this.heading = state.getHeadingVal();
        this.ang_vel = state.getAngVelVal();

        this.pos_var = state.getPosVar();
        this.heading_var = state.getHeadingVar();

    }
}