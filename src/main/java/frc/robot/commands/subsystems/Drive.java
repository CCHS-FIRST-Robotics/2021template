package frc.robot.commands.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.HardwareObjects;
import frc.robot.Constants;

public class Drive {

    public Drive() {

    }

    public void setDrives(double left_prop, double right_prop, HardwareObjects hardware) {
        hardware.LEFT_MOTOR.set(ControlMode.PercentOutput, left_prop);
        hardware.RIGHT_MOTOR.set(ControlMode.PercentOutput, right_prop * -1);
    }
}
