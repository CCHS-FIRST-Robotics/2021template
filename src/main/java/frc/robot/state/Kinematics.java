package frc.robot.state;

import frc.robot.Constants;
import frc.robot.helper.SimpleMat;
import java.lang.Math;

public class Kinematics {

    public double friction = Constants.INIT_FRICTION;

    public Values Val = new Values();
    public Variances Var = new Variances();

    public Kinematics() {
        this.friction = Constants.INIT_FRICTION;
    }

    public void predict(double dt) {
        this.Val.predict(dt);
        this.Var.predict(dt);
    }

    class Values {
        public double[] pos = Constants.INIT_POS;
        public double[] vel = Constants.INIT_VEL;
        public double[] acc = Constants.INIT_ACC;

        public double heading = Constants.INIT_HEADING;
        public double ang_vel = Constants.INIT_ANG_VEL;
        public double ang_acc = Constants.INIT_ANG_ACC;

        public Values() {
            this.pos = Constants.INIT_POS;
            this.vel = Constants.INIT_VEL;
            this.acc = Constants.INIT_ACC;

            this.heading = Constants.INIT_HEADING;
            this.ang_vel = Constants.INIT_ANG_VEL;

        }

        public void predict(double dt) {
            // FRICTION ACCEL
            double[] vel_unit = SimpleMat.unitVec(this.vel);
            double[] friction_a = { friction * Constants.GRAV_ACC * vel_unit[0],
                    friction * Constants.GRAV_ACC * vel_unit[1] };
            double vel_mag = SimpleMat.mag(this.vel);
            double friction_mag = SimpleMat.mag(friction_a);
            if (friction_mag * dt > vel_mag) {
                SimpleMat.scaleVec(friction_a, SimpleMat.mag(this.vel) / (dt * friction_mag));
            }
            // POS
            this.pos[0] = this.pos[0] + this.vel[0] * dt + 0.5 * (this.acc[0] + friction_a[0]) * dt * dt;
            this.pos[1] = this.pos[1] + this.vel[1] * dt + 0.5 * (this.acc[1] + friction_a[1]) * dt * dt;
            // VEL
            this.vel[0] = this.vel[0] + (this.acc[0] + friction_a[0]) * dt;
            this.vel[1] = this.vel[1] + (this.acc[1] + friction_a[1]) * dt;
            // HEADING
            double m_o_i = Constants.ROBOT_WIDTH * Constants.ROBOT_WIDTH * Constants.ROBOT_MASS * 0.125;
            double ang_fric = (this.ang_acc / (Math.abs(this.ang_acc) + 0.001)) * Constants.ROBOT_WIDTH
                    * Constants.GRAV_ACC * friction / (2 * m_o_i);

            this.heading = this.heading + this.ang_vel * dt + 0.5 * (this.ang_acc + ang_fric) * dt * dt;
            this.heading = this.heading % (2 * Math.PI);
            // ANG VEL
            if (Math.abs(this.ang_acc) < Math.abs(ang_fric)) {
                if (Math.abs(this.ang_acc + ang_fric) * dt > Math.abs(this.ang_vel)) {
                    this.ang_vel = 0;
                } else {
                    this.ang_vel = this.ang_vel + (this.ang_acc + ang_fric) * dt;
                }
            } else {
                this.ang_vel = this.ang_vel + (this.ang_acc + ang_fric) * dt;
            }

        }
    }

    class Variances {
        public double pos = Constants.INIT_VARIANCE;
        public double vel = Constants.INIT_VARIANCE;
        public double acc = Constants.INIT_VARIANCE;

        public double heading = Constants.INIT_VARIANCE;
        public double ang_vel = Constants.INIT_VARIANCE;
        public double ang_acc = Constants.INIT_VARIANCE;

        public Variances() {
            this.pos = Constants.INIT_VARIANCE;
            this.vel = Constants.INIT_VARIANCE;
            this.acc = Constants.INIT_VARIANCE;

            this.heading = Constants.INIT_VARIANCE;
            this.ang_vel = Constants.INIT_VARIANCE;
            this.ang_acc = Constants.INIT_VARIANCE;

        }

        public void predict(double dt) {
            this.pos = this.pos + this.vel * dt + 0.5 * this.acc * dt * dt;
            this.vel = this.vel + this.acc * dt;
            this.heading = this.heading + this.ang_vel * dt + 0.5 * this.ang_acc * dt * dt;
            this.ang_vel = this.ang_vel + this.ang_acc * dt;
        }
    }
}
