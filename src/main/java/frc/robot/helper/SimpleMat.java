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
            remainder = 2 * Math.PI - remainder;
        }
        return remainder;
    }
}
