package frc.robot.sensors;

import frc.robot.sensors.BaseSensor;
import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.helper.SimpleMat;

import com.ctre.phoenix.motorcontrol.can.*;
import java.lang.Math;
import frc.robot.HardwareObjects;

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

    public DriveEncoderSensor(double sync_time) {
        this.LAG_TIME = 0.0; // No Lag
        this.SYNC_TIME = sync_time;
        this.last_updated = sync_time;
    }

    public boolean shouldUse() {
        return true;
    }

    public boolean commonSense(double radss) {
        if (radss > 100) {
            return false;
        }
        if (radss < -100) {
            return false;
        }
        return true;
    }

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

    public void processValue(MainState state, HardwareObjects hardware) {

        double l_raw = hardware.LEFT_MOTOR1.getSelectedSensorVelocity(1);
        double r_raw = hardware.RIGHT_MOTOR1.getSelectedSensorVelocity(1);
        // Convert to 4096 units/rot / 100ms
        double l_radss = l_raw * 2 * Math.PI * -1 / (4096 * 0.33 * 8.35 / 10);
        double r_radss = r_raw * 2 * Math.PI / (4096 * 0.33 * 8.35 / 10);

        this.log_l_radss = l_radss;
        this.log_r_radss = r_radss;

        localDisplacement(l_radss, r_radss, Constants.MAIN_DT);

        double[] o_delta = { 0, 0 };
        o_delta = SimpleMat.rot2d(o_local_delta, this.heading);

        double[] pred_pos = { this.pos[0] + o_delta[0], this.pos[1] + o_delta[1] };

        double new_heading = this.heading + this.arc_angle;

        // compute variances
        double dist_coeff = (Math.abs(l_radss) + Math.abs(r_radss)) * Constants.MAIN_DT / (2 * Math.PI);
        double p_var = this.pos_var
                + Constants.VAR_RAD_VAR * dist_coeff * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC;

        double v_var = Constants.VAR_RAD_VAR * dist_coeff * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC
                / Constants.MAIN_DT;

        double diff_coeff = (Math.abs(l_radss - r_radss)) * Constants.MAIN_DT / (2 * Math.PI);
        double h_var = this.heading_var
                + Constants.VAR_RAD_VAR * 4 * diff_coeff * Constants.MAIN_DT / Constants.ROBOT_WIDTH;

        double h_ang_var = Constants.VAR_RAD_VAR * diff_coeff / Constants.ROBOT_WIDTH;

        // Pos
        double[] xpos = state.kalmanUpdate(state.getPosVal()[0], state.getPosVar(), pred_pos[0], p_var);
        double[] ypos = state.kalmanUpdate(state.getPosVal()[1], state.getPosVar(), pred_pos[1], p_var);
        double[] kpos = { xpos[0], ypos[1] };
        state.setPos(pred_pos, xpos[1]);

        // Vel
        double[] xvel = state.kalmanUpdate(state.getVelVal()[0], state.getVelVar(), o_delta[0] / Constants.MAIN_DT,
                v_var);
        double[] yvel = state.kalmanUpdate(state.getVelVal()[1], state.getVelVar(), o_delta[1] / Constants.MAIN_DT,
                v_var);
        double[] kvel = { xvel[0], yvel[0] };
        state.setVel(kvel, xvel[1]);

        // Heading
        double[] kheading = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), new_heading, h_var);
        this.log_h_pred = new_heading;
        this.log_hk_pred = kheading[0];
        state.setHeading(kheading[0], kheading[1]);

        // Ang Vel
        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(),
                this.arc_angle / Constants.MAIN_DT, h_ang_var);
        state.setAngVel(kangvel[0], kangvel[1]);

        this.pos = state.getPosVal();
        this.heading = state.getHeadingVal();
        this.ang_vel = state.getAngVelVal();

        this.pos_var = state.getPosVar();
        this.heading_var = state.getHeadingVar();

    }
}