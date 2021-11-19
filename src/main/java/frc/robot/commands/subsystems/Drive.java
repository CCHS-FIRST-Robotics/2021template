package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Drive {

    public Drive() {

    }

    /**
     * Takes power proportions for left and right wheels and feeds it into hardware
     * objects
     * 
     * @param left_prop  left motor power proportion from [-1,1]
     * @param right_prop right motor power proportion from [-1,1]
     * @param hardware   robot hardware objects
     */
    public void setDrives(double left_prop, double right_prop, HardwareObjects hardware) {
        hardware.LEFT_MOTOR1.set(ControlMode.PercentOutput, left_prop);
        hardware.LEFT_MOTOR2.set(ControlMode.PercentOutput, left_prop);
        hardware.RIGHT_MOTOR1.set(ControlMode.PercentOutput, right_prop * -1);
        hardware.RIGHT_MOTOR2.set(ControlMode.PercentOutput, right_prop * -1);
    }
}
