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
import frc.robot.helper.*;
import java.lang.Math;

import frc.robot.commands.Command;
import frc.robot.commands.CommandHelper;

public class Controller {
    XboxController xbox = new XboxController(Constants.XBOX_PORT);

    PID l_pid;
    PID r_pid;

    public Controller() {
        l_pid = new PID(Constants.C_BASE_GAIN, 0.0, 0.001);
        r_pid = new PID(Constants.C_BASE_GAIN, 0.0, 0.001);
    }

    public Command getCommands(MainState state) {
        // double stick sensitivity control schema
        // traditional stick controls, however, left stick is less sensitive in x
        // direction
        // right stick is less sensitive in y direction
        // triggers do tank drive
        double lx_prop = xbox.getX(Hand.kLeft);
        double ly_prop = xbox.getY(Hand.kLeft);

        double rx_prop = xbox.getX(Hand.kRight);
        double ry_prop = xbox.getY(Hand.kRight);

        double l_trig = xbox.getTriggerAxis(Hand.kLeft);
        double r_trig = xbox.getTriggerAxis(Hand.kRight);

        double l_bump_prop = 1;
        double r_bump_prop = 1;
        if (xbox.getBumper(Hand.kLeft)) {
            l_bump_prop = -1;
        }
        if (xbox.getBumper(Hand.kRight)) {
            r_bump_prop = -1;
        }

        double l_target = (ly_prop * -1 + lx_prop * 0.2) + (ry_prop * -0.2 + rx_prop * 1) + l_trig * l_bump_prop;
        double r_target = (ly_prop * -1 + lx_prop * -0.2) + (ry_prop * -0.2 + rx_prop * -1) + r_trig * r_bump_prop;

        double l_fac = Math.abs(l_target);
        double r_fac = Math.abs(r_target);
        if (l_fac > 1) {
            l_pid.setGain(l_fac * Constants.C_BASE_GAIN);
        } else {
            l_pid.setGain(Constants.C_BASE_GAIN);
        }
        if (r_fac > 1) {
            r_pid.setGain(r_fac * Constants.C_BASE_GAIN);
        } else {
            r_pid.setGain(Constants.C_BASE_GAIN);
        }
        l_target = Math.min(1, Math.max(-1, l_target)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;
        r_target = Math.min(1, Math.max(-1, r_target)) * Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60;

        double l_delta = l_target - state.getLWhlRadssVal();
        double r_delta = r_target - state.getRWhlRadssVal();

        double l_resp = SimpleMat.unitClamp(l_pid.update(l_delta));
        double r_resp = SimpleMat.unitClamp(r_pid.update(r_delta));

        Command command = new Command(l_resp, r_resp);
        // Logging
        return command;
    }
}
