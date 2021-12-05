package frc.robot.state;

import frc.robot.Constants;

/**
 * Wheel Class. Only stores wheel rpm
 */
public class Wheel {
    public Values Val = new Values();
    public Variances Var = new Variances();

    public Wheel() {

    }

    public void predict(double main_dt) {
    }

    class Values {
        public double left_wheel_radss;
        public double right_wheel_radss;

        public Values() {
            this.left_wheel_radss = Constants.INIT_WHL_RPM;
            this.right_wheel_radss = Constants.INIT_WHL_RPM;
        }
    }

    class Variances {
        public double left_wheel_radss;
        public double right_wheel_radss;

        public Variances() {
            this.left_wheel_radss = Constants.INIT_VARIANCE;
            this.right_wheel_radss = Constants.INIT_VARIANCE;
        }
    }
}
