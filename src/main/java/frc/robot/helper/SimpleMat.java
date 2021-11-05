package frc.robot.helper;

import java.lang.Math;

public class SimpleMat {
    public static double[] rot2d(double[] vec, double theta) {
        double[] prod = { 0, 0 };
        prod[0] = Math.cos(theta) * vec[0] - Math.sin(theta) * vec[1];
        prod[1] = Math.sin(theta) * vec[0] + Math.cos(theta) * vec[1];
        return prod;
    }

    public static double mag(double[] vec) {
        return Math.pow(vec[0] * vec[0] + vec[1] * vec[1], 0.5);
    }

    public static double[] unitVec(double[] vec) {
        double mag = Math.pow(vec[0] * vec[0] + vec[1] * vec[1], 0.5) + 0.001;
        double[] unit = { vec[0] / mag, vec[1] / mag };
        return unit;
    }

    public static double[] scaleVec(double[] vec, double scalar) {
        double[] newVec = { vec[0] * scalar, vec[1] * scalar };
        return newVec;
    }

    public static double scalarProject(double[] reference, double[] given) {
        double[] unit_ref = unitVec(reference);
        double projection = unit_ref[0] * given[0] + unit_ref[1] * given[1];
        return projection;
    }

    public static double[] projectHeading(double heading, double scalar) {
        double[] projected = { -1 * Math.sin(heading) * scalar, Math.cos(heading) * scalar };
        return projected;
    }

    public static double angleRectifier(double theta) {
        double remainder = theta % (2 * Math.PI);
        if (remainder > Math.PI) {
            remainder = remainder - 2 * Math.PI;
        }
        return remainder;
    }

    public static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1];
    }

    public static double vec2theta(double[] a) {
        double[] unit = unitVec(a);
        double theta = Math.acos(unit[1]);
        if (unit[0] > 0) {
            theta = Math.PI * 2 - theta;
        }
        return theta;
    }

    public static double vecsAngle(double[] a, double[] b) {
        double cos_val = dot(a, b) / (mag(a) * mag(b));
        double diff = Math.acos(cos_val);
        // determine if its left rot or right by calculating global vert theta
        double a_theta = vec2theta(a);
        double b_theta = vec2theta(b);

        double turn = b_theta - a_theta;

        double way1 = Math.abs(turn);
        double way2 = 2 * Math.PI - way1;
        if (way1 < way2) {
        } else {
            turn = turn * -1;
        }

        return turn;
    }

    public static double vecsAngle2(double[] heading, double[] y_p) {
        double turn_mag = Math.acos(dot(heading, y_p) / (mag(heading) * mag(y_p)));
        double[] new_head = { -1 * heading[1], heading[0] };
        double side = dot(new_head, y_p);
        double turn;
        if (side > 0) {
            turn = turn_mag;
        } else {
            turn = turn_mag * -1;
        }
        return turn;
    }

    public static double vectorDistance(double[] a, double[] b) {
        double[] new_vec = { a[0] - b[0], a[1] - b[1] };
        return mag(new_vec);
    }
}
