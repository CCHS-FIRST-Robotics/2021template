package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

public class HardwareObjects {
    public static final TalonSRX LEFT_MOTOR = new WPI_TalonSRX(Constants.L_TALON_PORT);
    public static final TalonSRX RIGHT_MOTOR = new WPI_TalonSRX(Constants.R_TALON_PORT);
}