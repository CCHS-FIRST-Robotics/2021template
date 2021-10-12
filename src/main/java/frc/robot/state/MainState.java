package frc.robot.state;
public class MainState {
    Kinematics Phy = new Kinematics();

    public void predict(double dt){
        this.Phy.predict(dt);
    }

    public static double[] kalmanUpdate(double current_val, double current_var, double sensed_val, double sensed_var){
        double kalman_gain = current_var/(current_var + sensed_var);
        double new_val = kalman_gain * sensed_val + (1 - kalman_gain) * current_val;
        double new_var = (1 - kalman_gain) * current_var;
        double[] new_1 = {new_val, new_var};
        return new_1;
    }
    //================
    //GET SET VALUES
    //================

    //pos
    public double[] getPosVal(){
        return this.Phy.Val.pos;
    }
    public double getPosVar(){
        return this.Phy.Var.pos;
    }
    public void setPosVar(double[] val, double var){
        this.Phy.Val.pos = val;
        this.Phy.Var.pos = var;
    }

    //vel
    public double[] getVelVal(){
        return this.Phy.Val.vel;
    }
    public double getVelVar(){
        return this.Phy.Var.vel;
    }
    public void setVelVar(double[] val, double var){
        this.Phy.Val.vel = val;
        this.Phy.Var.vel = var;
    }

    //acc
    public double[] getAccVal(){
        return this.Phy.Val.acc;
    }
    public double getAccVar(){
        return this.Phy.Var.acc;
    }
    public void setAccVar(double[] val, double var){
        this.Phy.Val.acc = val;
        this.Phy.Var.acc = var;
    }

    //heading
    public double getHeadingVal(){
        return this.Phy.Val.heading;
    }
    public double getHeadingVar(){
        return this.Phy.Var.heading;
    }
    public void setHeadingVar(double val, double var){
        this.Phy.Val.heading = val;
        this.Phy.Var.heading = var;
    }

    //ang_vel
    public double getAngVelVal(){
        return this.Phy.Val.ang_vel;
    }
    public double getAngVelVar(){
        return this.Phy.Var.ang_vel;
    }
    public void setAngVelVar(double val, double var){
        this.Phy.Val.ang_vel = val;
        this.Phy.Var.ang_vel = var;
    }

    //Left wheel ang vel
    public double getLWheelVelVal(){
        return this.Phy.Val.l_whl_vel;
    }
    public double getLWheelVelVar(){
        return this.Phy.Var.l_whl_vel;
    }
    public void setLWheelVelVar(double val, double var){
        this.Phy.Val.l_whl_vel = val;
        this.Phy.Var.l_whl_vel = var;
    }

    //Right wheel ang vel
    public double getRWheelVelVal(){
        return this.Phy.Val.r_whl_vel;
    }
    public double getRWheelVelVar(){
        return this.Phy.Var.r_whl_vel;
    }
    public void setRWheelVelVar(double val, double var){
        this.Phy.Val.r_whl_vel = val;
        this.Phy.Var.r_whl_vel = var;
    }
    
}