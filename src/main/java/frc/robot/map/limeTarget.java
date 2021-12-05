package frc.robot.map;

import frc.robot.state.MainState;
import frc.robot.helper.SimpleMat;

public class limeTarget {
    public double pos[] = { 0, 0 };
    public double arange[] = { 0, 6.28 };
    double x_fov = 0.6109;

    public limeTarget(double x_pos, double y_pos, double theta_0, double theta_1) {
        this.pos[0] = x_pos;
        this.pos[1] = y_pos;

        this.arange[0] = theta_0;
        this.arange[1] = theta_1;
    }

    public boolean isVisible(MainState state) {
        double[] difference = SimpleMat.subtract(state.getPosVal(), this.pos);
        double angle = 0;
        if (difference[1] == 0) {
            if (difference[0] < 0) {
                angle = Math.PI / 2;
            } else {
                angle = 3 * Math.PI / 2;
            }
        }
        if (difference[1] > 0) {
            angle = Math.atan(-1 * difference[0] / difference[1]);
            if (angle < 0) {
                angle = Math.PI * 2 + angle;
            }
        } else {
            angle = Math.atan(-1 * difference[0] / difference[1]);
            angle = angle + Math.PI;
        }
        if (angle < arange[1] && angle > arange[0]) {
            double[] d2 = SimpleMat.scaleVec(difference, SimpleMat.mag(difference));
            double[] head_vec = SimpleMat.projectHeading(state.getHeadingVal(), 1);
            double span_angle = Math.acos(SimpleMat.dot(head_vec, d2));
            if (span_angle < this.x_fov) {
                return true;
            }
        }
        return false;
    }

}
