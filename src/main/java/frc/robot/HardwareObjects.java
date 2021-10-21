package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

public class HardwareObjects {
    public TalonSRX LEFT_MOTOR;
    public TalonSRX RIGHT_MOTOR;

    public HardwareObjects() {
        LEFT_MOTOR = new WPI_TalonSRX(Constants.L_TALON_PORT);
        RIGHT_MOTOR = new WPI_TalonSRX(Constants.R_TALON_PORT);
    }
}