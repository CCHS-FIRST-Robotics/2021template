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
import frc.robot.commands.UpdateState;
import frc.robot.sensors.DriveEncoderSensor;
import frc.robot.sensors.IMUSensor;
import frc.robot.HardwareObjects;
import frc.robot.network.*;

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
  public IMUSensor imu_sensor = new IMUSensor(SYNC_TIME);
  public AI ai = new AI();
  public CommandHandler command_handler = new CommandHandler();
  public Command main_command = new Command(0, 0);
  public HardwareObjects hardware;
  public Network network;

  public RobotContainer() {
    this.main_state = new MainState();
    this.ai = new AI();
    this.command_handler = new CommandHandler();
    this.main_command = new Command(0, 0);
    SYNC_TIME = (double) main_timer.getTimeInMillis() / 1000;
    this.drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
    this.imu_sensor = new IMUSensor(SYNC_TIME);
    this.hardware = new HardwareObjects();
    this.network = new Network();

    reset();
  }

  public void init() {

  }

  public void reset() {
    this.imu_sensor.reset(this.hardware);
    this.main_command = new Command(0, 0);
    this.main_state = new MainState();
    this.network.init();
  }

  public void mainLoop() {
    this.main_command = this.ai.getCommand(this.main_state);

    UpdateState.updateState(this.main_state, this.main_command);
    this.command_handler.scheduleCommands(this.main_command, this.hardware);

    if (this.drive_encoder_sensor.shouldUse()) {
      this.drive_encoder_sensor.processValue(this.main_state, this.hardware);
    }
    if (this.imu_sensor.shouldUse(this.hardware)) {
      this.imu_sensor.processValue(this.main_state, this.hardware);
    }

    this.main_state.predict(Constants.MAIN_DT);

    // Logging
    this.network.writeNTable();
  }

  public void setControllerState() {
    this.ai.setControllerState();
  }

  public void setAutonomousState() {
    this.ai.setAutonomousState();
  }

}
