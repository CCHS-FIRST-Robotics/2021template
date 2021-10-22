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

    double arc_angle = 0;
    double o_local_delta[] = { 0, 0 };
    double VARIANCE = 0.05;

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
        double l = l_radss * Constants.WHEEL_RADIUS * Constants.INIT_L_WHL_TRAC;
        double r = r_radss * Constants.WHEEL_RADIUS * Constants.INIT_R_WHL_TRAC;

        this.arc_angle = dt * (r - l) / Constants.ROBOT_WIDTH;

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
        double l_raw = hardware.LEFT_MOTOR.getSelectedSensorVelocity(1);
        double r_raw = hardware.RIGHT_MOTOR.getSelectedSensorVelocity(1);
        // Convert to rads per sec
        double l_radss = l_raw * 2 * Math.PI * -1 / 60;
        double r_radss = r_raw * 2 * Math.PI / 60;

        localDisplacement(l_radss, r_radss, Constants.MAIN_DT);

        double[] o_delta = { 0, 0 };
        o_delta = SimpleMat.rot2d(o_local_delta, this.heading);

        double[] pred_pos = { this.pos[0] + o_delta[0], this.pos[1] + o_delta[1] };

        double new_heading = this.heading + this.arc_angle;

        // pos
        double[] xpos = state.kalmanUpdate(state.getPosVal()[0], state.getPosVar(), pred_pos[0], VARIANCE);
        double[] ypos = state.kalmanUpdate(state.getPosVal()[1], state.getPosVar(), pred_pos[1], VARIANCE);
        double[] kpos = { xpos[0], ypos[1] };
        state.setPos(kpos, xpos[1]);

        // Heading
        double[] kheading = state.kalmanUpdate(state.getHeadingVal(), state.getHeadingVar(), new_heading, 0.1);
        state.setHeading(kheading[0], kheading[1]);

        // Ang Vel
        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(),
                this.arc_angle / Constants.MAIN_DT, 0.1 / Constants.MAIN_DT);
        state.setAngVel(kangvel[0], kangvel[1]);

        pos = state.getPosVal();
        heading = state.getHeadingVal();
        ang_vel = state.getAngVelVal();
    }
}