package frc.robot.ai.subroutines;

import frc.robot.Constants;

public class TurnToPoint2 {
    double max_ang_vel;

    public TurnToPoint2() {
        double m_o_i = Constants.ROBOT_WIDTH * Constants.ROBOT_WIDTH * Constants.ROBOT_MASS * 0.125;
        this.max_ang_vel = (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS
                * Constants.INIT_L_WHL_TRAC / (Constants.ROBOT_WIDTH / 2);
        this.max_ang_acc = (Constants.MOTOR_MAX_TORQUE*2/Constants.WHEEL_RADIUS)/m_o_i;
    }

    double getCurveTime(double )

    double getAcceleration(double ang_vel, double delta) {
        // max vel l/r
        // find out if deccel element or accel 
        // pure deccel
        double deccel_grad = 
    }
}
