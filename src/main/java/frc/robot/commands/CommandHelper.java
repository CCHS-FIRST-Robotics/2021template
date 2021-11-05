package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import java.lang.Math;
import frc.robot.helper.SimpleMat;

public class CommandHelper {
    public static Command computeCommand(double left_target_rpm, double right_target_rpm) {
        double left_prop = left_target_rpm / (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60);
        if (left_prop > 1) {
            left_prop = 1;
        }
        if (left_prop < -1) {
            left_prop = -1;
        }
        double right_prop = right_target_rpm / (Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60);
        if (right_prop > 1) {
            right_prop = 1;
        }
        if (right_prop < -1) {
            right_prop = -1;
        }
        return new Command(left_prop, right_prop);
    }




    static double motorController(double target_rpm, double current_rpm) {
        double vtnew = target_rpm * Constants.WHEEL_RADIUS * Constants.INIT_R_WHL_TRAC;
        double vtold = current_rpm * Constants.WHEEL_RADIUS * Constants.INIT_R_WHL_TRAC
                - Constants.GRAV_ACC * Constants.INIT_FRICTION * Constants.MAIN_DT;
        double power = (Constants.ROBOT_MASS * (vtnew * vtnew - vtold * vtold)) / (4 * Constants.MAIN_DT);
        double prop = power / Constants.MOTOR_MAX_POWER;
        if (prop > 1) {
            prop = 1;
        }
        if (prop < -1) {
            prop = -1;
        }
        return prop;
    }

    static double inverseMotorController(double set_prop, double current_rpm, double whl_traction) {
        double vtold = current_rpm * Constants.WHEEL_RADIUS * Constants.INIT_R_WHL_TRAC
                - Constants.GRAV_ACC * Constants.INIT_FRICTION * Constants.MAIN_DT;
        double numer = (vtold * vtold)
                + (4 * Math.abs(set_prop) * Constants.MOTOR_MAX_POWER * Constants.MAIN_DT) / Constants.ROBOT_MASS;
        double direction = 0;
        if (numer < 0) {
            if (current_rpm > 0) {
                direction = -1;
            } else {
                direction = 1;
            }
        } else {
            if (current_rpm > 0) {
                direction = 1;
            } else {
                direction = -1;
            }
        }
        double new_rpm = direction * Math.pow(Math.abs(numer), 0.5) / (Constants.WHEEL_RADIUS * whl_traction);
        return new_rpm;
    }

    static double inverseMotorControllerVariance(double set_prop, double current_rpm_var, double whl_traction) {
        double set_prop_var = set_prop * Constants.MOTOR_PROP_VAR;
        double numer = (current_rpm_var * current_rpm_var)
                + (4 * Math.abs(set_prop_var) * Constants.MOTOR_MAX_POWER * Constants.MAIN_DT) / Constants.ROBOT_MASS;
        double new_var = Math.pow(Math.abs(numer), 0.5) / (Constants.WHEEL_RADIUS * whl_traction);
        return new_var;
    }

    static double motorPowerAcceleration(double set_energy) {
        return 2 * Math.pow(set_energy / (Constants.ROBOT_MASS * Constants.MAIN_DT), 0.5);
    }
}
