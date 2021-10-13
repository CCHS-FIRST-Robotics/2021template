package frc.robot.commands;
import frc.robot.state.MainState;
import frc.robot.Constants;
import java.lang.Math;
import frc.robot.helper.SimpleMat;
public class CommandHelper {
    public static Command computeCommand(MainState state, double left_target_rpm, double right_target_rpm){
        double left_prop = motorController(left_target_rpm, state.getLWheelVelVal());
        double right_prop = motorController(right_target_rpm, state.getRWheelVelVal());
        return new Command(left_prop, right_prop);
    }
    public static void updateState(MainState state, Command command){
        //Wheel RPM
        double left_rpm = inverseMotorController(command.left_pwr_prop, state.getLWheelVelVal(), Constants.INIT_L_WHL_TRAC);
        double right_rpm = inverseMotorController(command.right_pwr_prop, state.getRWheelVelVal(), Constants.INIT_R_WHL_TRAC);
        double left_var = inverseMotorControllerVariance(command.left_pwr_prop, state.getLWheelVelVar(), Constants.INIT_L_WHL_TRAC);
        double right_var = inverseMotorControllerVariance(command.right_pwr_prop, state.getRWheelVelVar(), Constants.INIT_R_WHL_TRAC);
        state.setLWheelVel(left_rpm, left_var);
        state.setRWheelVel(right_rpm, right_var);
        //Accel + Ang Vel
        double left_a = motorPowerAcceleration(Constants.MOTOR_MAX_POWER*command.left_pwr_prop);
        double right_a = motorPowerAcceleration(Constants.MOTOR_MAX_POWER*command.right_pwr_prop);
        double ang_vel = Constants.MAIN_DT*(right_a - left_a)/Constants.ROBOT_WIDTH;
        double ang = Constants.MAIN_DT*0.5*ang_vel;
        double[] local_a = {-0.5*Math.sin(ang)*(left_a + right_a),0.5*Math.cos(ang)*(left_a + right_a)};
        double[] global_a = SimpleMat.rot2d(local_a, state.getHeadingVal());
        double ave_prop_coeff = (Math.abs(command.left_pwr_prop) + Math.abs(command.right_pwr_prop))*0.5;
        state.setAngVel(ang,Constants.ANG_VEL_VARIANCE*ave_prop_coeff);
        state.setAcc(global_a,Constants.ACC_VARIANCE*ave_prop_coeff);
    }
    static double motorController(double target_rpm, double current_rpm){
        double vtnew = target_rpm*Constants.WHEEL_RADIUS*Constants.INIT_R_WHL_TRAC;
        double vtold = current_rpm*Constants.WHEEL_RADIUS*Constants.INIT_R_WHL_TRAC - Constants.GRAV_ACC*Constants.INIT_FRICTION*Constants.MAIN_DT;
        double power = (Constants.ROBOT_MASS * (vtnew*vtnew - vtold*vtold))/(4*Constants.MAIN_DT);
        double prop = power/Constants.MOTOR_MAX_POWER;
        if (prop>1){
            prop = 1;
        }
        if (prop<-1){
            prop = -1;
        }
        return prop;
    }
    static double inverseMotorController(double set_prop, double current_rpm, double whl_traction){
        double vtold = current_rpm*Constants.WHEEL_RADIUS*Constants.INIT_R_WHL_TRAC - Constants.GRAV_ACC*Constants.INIT_FRICTION*Constants.MAIN_DT;
        double numer = (vtold * vtold) + (4 * set_prop * Constants.MOTOR_MAX_POWER * Constants.MAIN_DT)/Constants.ROBOT_MASS;
        double new_rpm = Math.pow(numer,0.5)/(Constants.WHEEL_RADIUS*whl_traction);
        return new_rpm;
    }
    static double inverseMotorControllerVariance(double set_prop, double current_rpm_var, double whl_traction){
        double set_prop_var = set_prop * Constants.MOTOR_PROP_VAR;
        double numer = (current_rpm_var * current_rpm_var) + (4 * set_prop_var * Constants.MOTOR_MAX_POWER * Constants.MAIN_DT)/Constants.ROBOT_MASS;
        double new_var = Math.pow(numer,0.5)/(Constants.WHEEL_RADIUS*whl_traction);
        return new_var;
    }
    static double motorPowerAcceleration(double set_energy){
        return 2*Math.pow(set_energy/(Constants.ROBOT_MASS*Constants.MAIN_DT),0.5);
    }
}