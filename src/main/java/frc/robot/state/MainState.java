package frc.robot.state;

import java.lang.*;

/**
 * MainState that is a collection for every substate object that comprises the
 * robot. Core of robot program.
 * 
 * @author Ludwig Tay
 */
public class MainState {
    Kinematics Phy = new Kinematics();
    Wheel Whl = new Wheel();
    WheelOdo WhlOdo = new WheelOdo();

    /**
     * predict all subclass states for dt time in the future.
     * 
     * @param dt time to predict in seconds.
     */
    public void predict(double dt) {
        this.Phy.predict(dt);
    }

    /**
     * Kalman Update. Main method of fusing sensor values by comparing variances.
     * 
     * @param current_val Current value from state.
     * @param current_var Current variance from state.
     * @param sensed_val  Sensed value from sensor.
     * @param sensed_var  Sensed variance from sensor.
     * @return Double len 2 array. Element 0 is fused value, Element 1 is fused
     *         variance.
     */
    public double[] kalmanUpdate(double current_val, double current_var, double sensed_val, double sensed_var) {
        double kalman_gain = current_var / (current_var + sensed_var + 0.0001);
        double new_val = kalman_gain * sensed_val + (1 - kalman_gain) * current_val;
        double new_var = kalman_gain * sensed_var + (1 - kalman_gain) * current_var;
        if (Double.isNaN(new_val) || Double.isNaN(new_var)) {
            double[] new_1 = { current_val, current_var };
            return new_1;
        } else {
            double[] new_1 = { new_val, new_var };
            return new_1;
        }
    }

    /**
     * Kalman Update. Main method of fusing sensor values by comparing variances.
     * Special version for wrapping angular values like heading.
     * 
     * @param current_val Current value from state.
     * @param current_var Current variance from state.
     * @param sensed_val  Sensed value from sensor.
     * @param sensed_var  Sensed variance from sensor.
     * @return Double len 2 array. Element 0 is fused value, Element 1 is fused
     *         variance.
     */
    public double[] kalmanAngleUpdate(double current_val, double current_var, double sensed_val, double sensed_var) {
        double[] candidate_sval = { sensed_val - 2 * Math.PI, sensed_val, sensed_val + 2 * Math.PI };
        double new_sval = sensed_val;
        double closest = 8 * Math.PI;
        for (int i = 0; i < 3; i++) {
            double dist = Math.abs(candidate_sval[i] - current_val);
            if (dist < closest) {
                closest = dist;
                new_sval = candidate_sval[i];
            }
        }
        double kalman_gain = current_var / (current_var + sensed_var);
        double new_val = kalman_gain * new_sval + (1 - kalman_gain) * current_val;
        double new_var = kalman_gain * sensed_var + (1 - kalman_gain) * current_var;
        if (Double.isNaN(new_val) || Double.isNaN(new_var)) {
            double[] new_1 = { current_val, current_var };
            return new_1;
        } else {
            double[] new_1 = { new_val, new_var };
            return new_1;
        }
    }
    // ================
    // GET SET VALUES
    // ================

    // pos
    public double[] getPosVal() {
        return this.Phy.Val.pos;
    }

    public double getPosVar() {
        return this.Phy.Var.pos;
    }

    public void setPos(double[] val, double var) {
        this.Phy.Val.pos = val;
        this.Phy.Var.pos = var;
    }

    // vel
    public double[] getVelVal() {
        return this.Phy.Val.vel;
    }

    public double getVelVar() {
        return this.Phy.Var.vel;
    }

    public void setVel(double[] val, double var) {
        this.Phy.Val.vel = val;
        this.Phy.Var.vel = var;
    }

    // acc
    public double[] getAccVal() {
        return this.Phy.Val.acc;
    }

    public double getAccVar() {
        return this.Phy.Var.acc;
    }

    public void setAcc(double[] val, double var) {
        this.Phy.Val.acc = val;
        this.Phy.Var.acc = var;
    }

    // heading
    public double getHeadingVal() {
        return this.Phy.Val.heading;
    }

    public double getHeadingVar() {
        return this.Phy.Var.heading;
    }

    public void setHeading(double val, double var) {
        this.Phy.Val.heading = val;
        this.Phy.Var.heading = var;
    }

    // ang_vel
    public double getAngVelVal() {
        return this.Phy.Val.ang_vel;
    }

    public double getAngVelVar() {
        return this.Phy.Var.ang_vel;
    }

    public void setAngVel(double val, double var) {
        this.Phy.Val.ang_vel = val;
        this.Phy.Var.ang_vel = var;
    }

    // ang_acc
    public double getAngAccVal() {
        return this.Phy.Val.ang_acc;
    }

    public double getAngAccVar() {
        return this.Phy.Var.ang_acc;
    }

    public void setAngAcc(double val, double var) {
        this.Phy.Val.ang_acc = val;
        this.Phy.Var.ang_acc = var;
    }

    // Wheel RPM
    public double getLWhlRadssVal() {
        return this.Whl.Val.left_wheel_radss;
    }

    public double getLWhlRadssVar() {
        return this.Whl.Var.left_wheel_radss;
    }

    public void setLWhlRadss(double val, double var) {
        this.Whl.Val.left_wheel_radss = val;
        this.Whl.Var.left_wheel_radss = var;
    }

    public double getRWhlRadssVal() {
        return this.Whl.Val.right_wheel_radss;
    }

    public double getRWhlRadssVar() {
        return this.Whl.Var.right_wheel_radss;
    }

    public void setRWhlRadss(double val, double var) {
        this.Whl.Val.right_wheel_radss = val;
        this.Whl.Var.right_wheel_radss = var;
    }

    public double[] getWhlOdoPosVal() {
        return this.WhlOdo.Val.pos;
    }

    public double getWhlOdoPosVar() {
        return this.WhlOdo.Var.whl_odo_pos_var;
    }

    public void setWhlOdoPos(double[] val, double var) {
        this.WhlOdo.Val.pos = val;
        this.WhlOdo.Var.whl_odo_pos_var = var;
    }

    public double getWhlOdoHVal() {
        return this.WhlOdo.Val.heading;
    }

    public double getWhlOdoHVar() {
        return this.WhlOdo.Var.heading_var;
    }

    public void setWhlOdoH(double val, double var) {
        this.WhlOdo.Val.heading = val;
        this.WhlOdo.Var.heading_var = var;
    }
}