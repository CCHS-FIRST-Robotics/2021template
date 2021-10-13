package frc.robot.commands.hardware;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Drive {
    TalonSRX left_talon = new TalonSRX(Constants.L_TALON_PORT);
    TalonSRX right_talon = new TalonSRX(Constants.R_TALON_PORT);
    public Drive(){
        this.left_talon = new TalonSRX(Constants.L_TALON_PORT);
        this.right_talon = new TalonSRX(Constants.R_TALON_PORT);
        
        this.left_talon.configFactoryDefault();
        this.right_talon.configFactoryDefault();
        
    }
    public void setDrives(double left_prop, double right_prop){
        this.left_talon.set(ControlMode.PercentOutput, left_prop);
        this.right_talon.set(ControlMode.PercentOutput,right_prop);
    }
}
