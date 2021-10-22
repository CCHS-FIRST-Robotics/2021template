package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

public class HardwareObjects {
    public TalonSRX LEFT_MOTOR1;
    public VictorSPX LEFT_MOTOR2;
    public TalonSRX RIGHT_MOTOR1;
    public TalonSRX RIGHT_MOTOR2;
    public PigeonIMU IMU;

    public HardwareObjects() {
        LEFT_MOTOR1 = new WPI_TalonSRX(Constants.L_TALON_PORT1);
        LEFT_MOTOR2 = new WPI_VictorSPX(Constants.L_VICTOR_PORT2);
        RIGHT_MOTOR1 = new WPI_TalonSRX(Constants.R_TALON_PORT1);
        RIGHT_MOTOR2 = new WPI_TalonSRX(Constants.R_TALON_PORT2);
        IMU = new PigeonIMU(Constants.IMU_PORT);

        LEFT_MOTOR1.configFactoryDefault();
        RIGHT_MOTOR1.configFactoryDefault();
        LEFT_MOTOR2.configFactoryDefault();
        RIGHT_MOTOR2.configFactoryDefault();
        IMU.configFactoryDefault();
        IMU.setFusedHeading(0.0, Constants.TIMEOUT_MS);
    }
}