package frc.robot.state;
import frc.robot.Constants;
import frc.robot.helper.SimpleMat;
import java.lang.Math;
public class Kinematics {

    public double mix = Constants.INIT_MIX;
    public double delta_weight = Constants.INIT_DW;
    public double friction = Constants.INIT_FRICTION;
    public double l_whl_trac = Constants.INIT_L_WHL_TRAC;
    public double r_whl_trac = Constants.INIT_R_WHL_TRAC;

    public Values Val = new Values();
    public Variances Var = new Variances();
    
    public Kinematics(){
        this.mix = Constants.INIT_MIX;
        this.delta_weight = Constants.INIT_DW;
        this.friction = Constants.INIT_FRICTION;
        this.l_whl_trac = Constants.INIT_L_WHL_TRAC;
        this.r_whl_trac = Constants.INIT_R_WHL_TRAC;
    }

    public void predict(double dt){
        this.Val.predict(dt);
        this.Var.predict(dt);   
    }
    class Values {
        public double[] pos = Constants.INIT_POS;
        public double[] vel = Constants.INIT_VEL;
        public double[] acc = Constants.INIT_ACC;

        public double heading = Constants.INIT_HEADING;
        public double ang_vel = Constants.INIT_ANG_VEL;

        public double l_whl_vel = Constants.INIT_L_WHL_VEL;
        public double r_whl_vel = Constants.INIT_R_WHL_VEL;



        public Values(){
            this.pos = Constants.INIT_POS;
            this.vel = Constants.INIT_VEL;
            this.acc = Constants.INIT_ACC;
    
            this.heading = Constants.INIT_HEADING;
            this.ang_vel = Constants.INIT_ANG_VEL;
    
            this.l_whl_vel = Constants.INIT_L_WHL_VEL;
            this.r_whl_vel = Constants.INIT_R_WHL_VEL;
    
        }

        public void predict(double dt){
            //==============
            //WHEEL ODOMOTRY CALCULATIONS
            //==============
            double l = this.l_whl_vel*Constants.WHEEL_RADIUS * l_whl_trac;
            double r = this.r_whl_vel*Constants.WHEEL_RADIUS * l_whl_trac;

            double arc_angle = dt*(r-l)/Constants.ROBOT_WIDTH;

            double[] o_local_delta = {0,0};
            if (r==l){
                o_local_delta[0] = 0;
                o_local_delta[1] = r;
            }
            else{
                double travel_mag = Constants.ROBOT_WIDTH*(l+r)/(2*(r-l));
                o_local_delta[0] = travel_mag * (Math.cos(arc_angle) - 1);
                o_local_delta[1] = travel_mag * (Math.sin(arc_angle)    );
            }
            double[] o_delta = SimpleMat.rot2d(o_local_delta, this.heading);

            double[] temp_vel = {this.vel[0], this.vel[1]};
            double temp_heading = this.heading;
            //FRICTION ACCEL
            double[] vel_unit = SimpleMat.unitVec(this.vel);
            double[] friction_a = {friction * Constants.GRAV_ACC * vel_unit[0], friction * Constants.GRAV_ACC * vel_unit[1]};
            //POS
            this.pos[0] = this.pos[0] 
                + mix * (this.vel[0]*dt + 0.5 * (this.acc[0] + friction_a[0]) * dt * dt) 
                + (1 - mix) * o_delta[0];
            this.pos[1] = this.pos[1] 
                + mix * (this.vel[1]*dt + 0.5 * (this.acc[1] + friction_a[1]) * dt * dt)
                + (1 - mix) * o_delta[1];
            //VEL
            this.vel[0] = this.vel[0] 
                + mix * ((this.acc[0] - friction_a[0]) * dt) 
                + (1 - mix) * (o_delta[0]/dt);
            this.vel[1] = this.vel[1] 
                + mix * ((this.acc[1] - friction_a[1]) * dt) 
                + (1 - mix) * (o_delta[1]/dt);
            //ACCEL
            this.acc[0] = delta_weight * this.acc[0] 
                + (1 - delta_weight) * (this.vel[0] - temp_vel[0])/dt;
            this.acc[1] = delta_weight * this.acc[1] 
                + (1 - delta_weight) * (this.vel[1] - temp_vel[1])/dt;
            //HEADING
            this.heading = this.heading + mix * (this.ang_vel * dt) + (1 - mix) * arc_angle;
            //ANG VEL
            this.ang_vel = delta_weight * this.ang_vel + (1 - delta_weight) * (this.heading - temp_heading)/dt;
        }
    }
    class Variances {
        public double pos = Constants.INIT_VARIANCE;
        public double vel = Constants.INIT_VARIANCE;
        public double acc = Constants.INIT_VARIANCE;

        public double heading = Constants.INIT_VARIANCE;
        public double ang_vel = Constants.INIT_VARIANCE;

        public double l_whl_vel = Constants.INIT_VARIANCE;
        public double r_whl_vel = Constants.INIT_VARIANCE;



        public Variances(){
            this.pos = Constants.INIT_VARIANCE;
            this.vel = Constants.INIT_VARIANCE;
            this.acc = Constants.INIT_VARIANCE;
    
            this.heading = Constants.INIT_VARIANCE;
            this.ang_vel = Constants.INIT_VARIANCE;
    
            this.l_whl_vel = Constants.INIT_VARIANCE;
            this.r_whl_vel = Constants.INIT_VARIANCE;
    
        }
        public void predict(double dt){
            double l = this.l_whl_vel*Constants.WHEEL_RADIUS * l_whl_trac;
            double r = this.r_whl_vel*Constants.WHEEL_RADIUS * r_whl_trac;
            double arc_angle = dt*(r-l)/Constants.ROBOT_WIDTH;
            this.pos = this.pos
                + mix * (this.vel * dt + 0.5 * this.acc * dt * dt)
                + (1-mix) * (l + r);
            this.vel = this.vel + this.acc * dt;
            this.heading = this.heading 
                + mix * this.ang_vel*dt
                + (1-mix) * arc_angle;
        }
    }
}
