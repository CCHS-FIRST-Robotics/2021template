package frc.robot.ai.finite_state;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants;
import frc.robot.state.MainState;
import java.lang.Math;

import frc.robot.commands.Command;
import frc.robot.commands.CommandHelper;

public class Controller {
    XboxController xbox = new XboxController(Constants.XBOX_PORT);
    public Controller(){
    }
    public Command getCommands(MainState state){
        double x_prop = xbox.getX(Hand.kLeft);
        double y_prop = xbox.getY(Hand.kLeft);
        double l_avel = Math.min(1,Math.max(-1,y_prop + x_prop))*Constants.MOTOR_MAX_RPM*2*Math.PI/60;
        double r_avel = Math.min(1,Math.max(-1,y_prop - x_prop))*Constants.MOTOR_MAX_RPM*2*Math.PI/60;
        Command command = CommandHelper.computeCommand(state, l_avel, r_avel);

        //Logging
        System.out.println("Left desired angular velocity: " + String.valueOf(l_avel));
        System.out.println("Right desired angular velocity: " + String.valueOf(r_avel));
        return command;
    }
}
