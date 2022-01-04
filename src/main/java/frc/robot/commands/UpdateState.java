package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.helper.SimpleMat;

public class UpdateState {
    public static double wheelForceFactor(double prop, double rel_vel) {
        double maxms = (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS;
        double factor = 1 - (rel_vel / maxms);
        if (factor > 1) {
            factor = 1;
        } else if (factor < 0) {
            factor = 0;
        }
        return factor;
    }

    public static double motorForce(MainState state, double pwr_prop) {
        double[] h_vec = SimpleMat.projectHeading(state.getHeadingVal(), 1);
        double rel_vel = SimpleMat.scalarProject(h_vec, state.getVelVal());

        double f = pwr_prop * Constants.MOTOR_MAX_TORQUE * wheelForceFactor(pwr_prop, rel_vel) / Constants.WHEEL_RADIUS;
        return f;
    }

    public static void updateState(MainState state, Command command) {
        // Accel + Ang Vel
        double m_o_i = Constants.ROBOT_WIDTH * Constants.ROBOT_WIDTH * Constants.ROBOT_MASS * 0.125;

        double left_f = motorForce(state, command.left_pwr_prop);
        double right_f = motorForce(state, command.right_pwr_prop);

        double torque = (right_f - left_f) * Constants.ROBOT_WIDTH * 0.5;
        double forward_f = (right_f + left_f);

        double ang_acc = torque / m_o_i;

        double[] acc = SimpleMat.projectHeading(state.getHeadingVal(), forward_f / Constants.ROBOT_MASS);

        double ave_prop_coeff = (Math.abs(command.left_pwr_prop) + Math.abs(command.right_pwr_prop)) * 0.5;
        state.setAngAcc(ang_acc, Constants.ANG_VEL_VARIANCE * ave_prop_coeff);
        state.setAcc(acc, Constants.ACC_VARIANCE * ave_prop_coeff);
    }
}
