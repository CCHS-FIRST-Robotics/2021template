package frc.robot.state;

import frc.robot.Constants;

public class WheelOdo {

    public Values Val = new Values();
    public Variances Var = new Variances();

    public WheelOdo() {

    }

    class Values {
        public double[] pos = { 0, 0 };
        public double heading = 0;

        public Values() {
        }

        public void predict(double dt) {

        }
    }
    class Variances {
        public double whl_odo_pos_var = 0.1;
        public double heading_var = 0.1;


        public Variances() { 

        }
        public void predict(){
            
        }
    }
}
