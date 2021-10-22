package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

public class HardwareObjects {
    public TalonSRX LEFT_MOTOR;
    public TalonSRX RIGHT_MOTOR;
    public PigeonIMU IMU;

    public HardwareObjects() {
        LEFT_MOTOR = new WPI_TalonSRX(Constants.L_TALON_PORT);
        RIGHT_MOTOR = new WPI_TalonSRX(Constants.R_TALON_PORT);
        IMU = new PigeonIMU(Constants.IMU_PORT);

        LEFT_MOTOR.configFactoryDefault();
        RIGHT_MOTOR.configFactoryDefault();
        IMU.configFactoryDefault();
        IMU.setFusedHeading(0.0, Constants.TIMEOUT_MS);
    }
}