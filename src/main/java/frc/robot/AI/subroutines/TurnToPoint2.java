package frc.robot.ai.subroutines;

import frc.robot.Constants;

public class TurnToPoint2 {
    double max_ang_vel;

    public TurnToPoint2() {
        this.max_ang_vel = (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS
                * Constants.INIT_L_WHL_TRAC / (Constants.ROBOT_WIDTH / 2);
        this.max_ang_acc = 
    }

    double getAcceleration(double ang_vel_mag, double delta_mag) {
        // max vel l/r
    }
}
