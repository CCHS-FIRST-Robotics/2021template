package frc.robot.state;

import java.lang.*;

public class MainState {
    Kinematics Phy = new Kinematics();

    public void predict(double dt) {
        this.Phy.predict(dt);
    }

    public double[] kalmanUpdate(double current_val, double current_var, double sensed_val, double sensed_var) {
        double kalman_gain = current_var / (current_var + sensed_var);
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
}