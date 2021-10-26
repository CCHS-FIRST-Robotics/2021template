// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ==================
    // HARDWARE PORTS
    // ==================
    public static final int R_TALON_PORT2 = 6;
    public static final int L_TALON_PORT1 = 9;

    public static final int R_TALON_PORT1 = 1;
    public static final int L_VICTOR_PORT2 = 2;

    public static final int IMU_PORT = 6;

    public static final int XBOX_PORT = 0;

    // ==================
    // STATE INITIALIZATION
    // ==================
    public static final double[] INIT_POS = { 0, 0 };
    public static final double[] INIT_VEL = { 0, 0 };
    public static final double[] INIT_ACC = { 0, 0 };

    public static final double INIT_HEADING = 0;
    public static final double INIT_ANG_VEL = 0;
    public static final double INIT_ANG_ACC = 0;

    public static final double INIT_L_WHL_VEL = 0;
    public static final double INIT_R_WHL_VEL = 0;

    public static final double INIT_L_WHL_TRAC = 0.9;
    public static final double INIT_R_WHL_TRAC = 0.9;

    public static final double INIT_FRICTION = 1.3;

    public static final double INIT_MIX = 0.5;

    public static final double INIT_DW = 0.4;

    public static final double INIT_VARIANCE = 0.01;
    // ==================
    // VARIANCE LINEAR APPROX
    // ==================
    public static final double ANG_VEL_VARIANCE = 0.5;
    public static final double ACC_VARIANCE = 0.1;

    // ==================
    // ENCODER VARIANCE
    // ==================
    public static final double VAR_RAD_VAR = 0.004;

    // ==================
    // IMU VARIANCE
    // ==================
    public static final double BASE_HEADING_VAR = 0.1;
    public static final double DELTA_VAR = 0.001;
    public static final double MAX_HEADING_VAR = 0.2;

    public static final double IMU_ACC_VAR = 0.05;
    // ==================
    // TIME CONSTANTS
    // ==================
    public static final double MAIN_DT = 0.02;

    // ==================
    // MOTOR CONSTANTS
    // ==================
    public static final double MOTOR_MAX_POWER = 674; // OUTPUT IN WATTS

    public static final double MOTOR_MAX_TORQUE = 4.82;

    public static final double MOTOR_MAX_RPM = 666;

    public static final double MOTOR_PROP_VAR = 0.1;

    public static final int TIMEOUT_MS = 50;

    // ==================
    // CERTAIN PHYSICAL CONSTANTS
    // ==================
    public static final double ROBOT_WIDTH = 0.635;
    public static final double WHEEL_RADIUS = 0.09;
    public static final double ROBOT_MASS = 40;

    public static final double GRAV_ACC = -9.81;

}
