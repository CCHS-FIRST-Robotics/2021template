// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;
import java.util.Calendar;
import frc.robot.commands.CommandHandler;
import frc.robot.ai.AI;
import frc.robot.commands.Command;
import frc.robot.commands.CommandHelper;
import frc.robot.state.MainState;
import frc.robot.commands.UpdateState;
import frc.robot.sensors.DriveEncoderSensor;
import frc.robot.sensors.IMUSensor;
import frc.robot.HardwareObjects;
import frc.robot.network.*;

import static frc.robot.Constants.*;

/**
 * RobotContainer class, contains every class of the robot and is constantly
 * running.
 * 
 * @author Ludwig Tay
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

  double log;

  /**
   * RobotContainer Constructor.
   */
  public RobotContainer() {
    this.main_state = new MainState();
    this.ai = new AI();
    this.command_handler = new CommandHandler();
    this.main_command = new Command(0, 0);
    this.SYNC_TIME = (double) main_timer.getTimeInMillis() / 1000;
    this.drive_encoder_sensor = new DriveEncoderSensor(SYNC_TIME);
    this.imu_sensor = new IMUSensor(SYNC_TIME);
    this.hardware = new HardwareObjects();
    this.network = new Network();

    reset();
  }

  public void init() {

  }

  /**
   * Reset values in robot container.
   */
  public void reset() {
    this.imu_sensor.reset(this.hardware);
    this.main_command = new Command(0, 0);
    this.main_state = new MainState();
    this.network.init(SYNC_TIME);
  }

  /**
   * mainLoop that executes state update, predict, ai and command scheduling.
   */
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

    this.network.writeNTable(this.main_state);

    this.log = this.network.stereo_net.getHeadingVal();

  }

  /**
   * Sets finite state in AI to controller.
   */
  public void setControllerState() {
    this.ai.setControllerState();
  }

  /**
   * Sets finite state in AI to auton.
   */
  public void setAutonomousState() {
    this.ai.setAutonomousState();
  }

}
