// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.CommandHandler;
import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.commands.CommandHelper;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public MainState main_state = new MainState();
  //public SomeSensor some_sensor = new SomeSensor();
  //public AI 
  public CommandHandler command_handler = new CommandHandler();
  public Command main_command = new Command(0,0);
  public RobotContainer(){
    this.main_state = new MainState();
    //Init somesensor and AI objects
    this.command_handler = new CommandHandler();
    this.main_command = new Command(0,0);
  }
  public void init(){
    
  }
  public void mainLoop(){
    //if this.some_sensor.canUse()
    //this.some_sensor.updateState(this.main_state)
    //this.main_command = this.AI.getCommand(this.main_state)
    CommandHelper.updateState(this.main_state, this.main_command);
    this.command_handler.scheduleCommands(this.main_command);
    this.main_state.predict(Constants.MAIN_DT);
  }
}
