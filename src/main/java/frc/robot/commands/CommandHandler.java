package frc.robot.commands;

import frc.robot.state.MainState;
import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.commands.subsystems.Drive;

import java.lang.Math;
import frc.robot.helper.SimpleMat;
import frc.robot.HardwareObjects;

public class CommandHandler {
    // Define devices as atrributes
    private Drive drive = new Drive();

    public CommandHandler() {
        this.drive = new Drive();
    }

    public void scheduleCommands(Command command, HardwareObjects hardware) {
        // Schedule hardware commands using command
        this.drive.setDrives(command.left_pwr_prop, command.right_pwr_prop, hardware);
    }
}
