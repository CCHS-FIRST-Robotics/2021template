// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.helper.Logging;
import java.io.File;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private RobotContainer robotContainer;
  Logging log = new Logging();
  File myObj;

  public void loggingConfigure(RobotContainer rContainer) {
    double[] accv = rContainer.imu_sensor.log_acc;
    String[] name = { "IMU Fused Heading", "Yaw Rate", "Pitch", "Acceleration 0", "Acceleration 1", "L Encoder Radss",
        "R Encoder Radss", "Predicted Heading", "Wheel Odo Heading", "Wheel Odo Heading (k)", "Predicted X Pos",
        "Predicted Y Pos" };
    double[] value = { rContainer.imu_sensor.log_fused_heading, rContainer.imu_sensor.log_yaw_vel,
        rContainer.imu_sensor.log_pitch, accv[0], accv[1], rContainer.drive_encoder_sensor.log_l_radss,
        rContainer.drive_encoder_sensor.log_r_radss, rContainer.main_state.getHeadingVal(),
        rContainer.drive_encoder_sensor.log_h_pred, rContainer.drive_encoder_sensor.log_hk_pred,
        rContainer.main_state.getPosVal()[0], rContainer.main_state.getPosVal()[1] };
    log.printInfo(name, value);
    System.out.println("Current State : " + rContainer.ai.getFiniteState());

    if (myObj.exists()) {
      System.out.println("File name: " + myObj.getName());
      System.out.println("Absolute path: " + myObj.getAbsolutePath());
    } else {
      System.out.println("The file does not exist.");
    }
  }

  public Robot() {
    this.myObj = new File("ai/autonomous_commands/auton1.txt");
    this.robotContainer = new RobotContainer();
    addPeriodic(() -> {
      this.robotContainer.mainLoop();
    }, Constants.MAIN_DT);

    addPeriodic(() -> {
      System.out.println(log.getVersion());
    }, 5);

    addPeriodic(() -> {
      loggingConfigure(this.robotContainer);
    }, 1);
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    this.robotContainer.init();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    this.robotContainer.setAutonomousState();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    this.robotContainer.reset();
    this.robotContainer.setControllerState();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
