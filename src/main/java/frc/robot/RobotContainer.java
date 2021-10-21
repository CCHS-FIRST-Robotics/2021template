// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;
import java.util.Calendar;
import frc.robot.commands.CommandHandler;
import frc.robot.ai.AI;
import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.commands.CommandHelper;
import frc.robot.sensors.DriveEncoderSensor;
import frc.robot.HardwareObjects;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public MainState main_state = new MainState();
  // public SomeSensor some_sensor = new SomeSensor();
  public Calendar main_timer = Calendar.getInstance();
  double SYNC_TIME = 0;
  public DriveEncoderSensor drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
  public AI ai = new AI();
  public CommandHandler command_handler = new CommandHandler();
  public Command main_command = new Command(0, 0);
  public HardwareObjects hardware;

  public RobotContainer() {
    this.main_state = new MainState();
    this.ai = new AI();
    this.command_handler = new CommandHandler();
    this.main_command = new Command(0, 0);
    SYNC_TIME = (double) main_timer.getTimeInMillis() / 1000;
    this.drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
    this.hardware = new HardwareObjects();
  }

  public void init() {

  }

  public void mainLoop() {
    if (this.drive_encoder_sensor.shouldUse()) {
      this.drive_encoder_sensor.processValue(this.main_state, this.hardware);
    }
    this.main_command = this.ai.getCommand(this.main_state);
    CommandHelper.updateState(this.main_state, this.main_command);
    this.command_handler.scheduleCommands(this.main_command, this.hardware);
    this.main_state.predict(Constants.MAIN_DT);

    // Logging
    System.out.println("Left Power Proportion: " + String.valueOf(this.main_command.left_pwr_prop));
    System.out.println("Right Power Proportion: " + String.valueOf(this.main_command.right_pwr_prop));
    System.out.println("Predicted Heading: " + String.valueOf(this.main_state.getHeadingVal()));
    System.out.println("Predicted Position: (" + String.valueOf(this.main_state.getPosVal()[0]) + ", "
        + String.valueOf(this.main_state.getPosVal()[1]) + ")");
    System.out.println("===========================");
  }

  public void setControllerState() {
    this.ai.setControllerState();
  }

  public void setAutonomousState() {
    this.ai.setAutonomousState();
  }

}
