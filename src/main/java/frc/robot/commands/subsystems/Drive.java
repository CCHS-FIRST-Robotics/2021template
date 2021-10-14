package frc.robot.commands.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Drive {
    public Drive(){
        HardwareObjects.LEFT_MOTOR.configFactoryDefault();
        HardwareObjects.RIGHT_MOTOR.configFactoryDefault();
        
    }
    public void setDrives(double left_prop, double right_prop){
        HardwareObjects.LEFT_MOTOR.set(ControlMode.PercentOutput, left_prop);
        HardwareObjects.RIGHT_MOTOR.set(ControlMode.PercentOutput,right_prop);
    }
}