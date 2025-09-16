// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  private PS4Controller controle;

  private DigitalInput sensotic;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    disabledTimer = new Timer();

    sensotic = new DigitalInput(0);

    controle = new PS4Controller(0);
    // SmartDashboard.putNumber("Tempo de partida", DriverStation.getMatchTime());

    UsbCamera camera = CameraServer.startAutomaticCapture();

    camera.setFPS(30);

    camera.setResolution(640, 480);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    // m_robotContainer.elevador.dashboard();
    // m_robotContainer.garra.dashboard();
    // m_robotContainer.Intake.dashboard();
    // SmartDashboard.putNumber("X", LimeLightHelpers.getBotPose_TargetSpace("")[2]);
    // SmartDashboard.putNumber("Y", LimeLightHelpers.getBotPose_TargetSpace("")[0]);
    // SmartDashboard.putNumber("rot", LimeLightHelpers.getBotPose_TargetSpace("")[4]);
    SmartDashboard.putNumber("Bateria", RobotController.getBatteryVoltage());
    // SmartDashboard.putNumber("ID april Tag", LimeLightHelpers.getFiducialID(""));
    SmartDashboard.putNumber("Pose X", m_robotContainer.drivebase.posex());

    m_robotContainer.garra.dashboard();
    // m_robotContainer.elevador.dashboard();
    // m_robotContainer.Intake.dashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * 
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println("Pose robo: " + m_robotContainer.drivebase.getPose());
    m_robotContainer.drivebase.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // m_robotContainer.elevador.setElevador();
    // m_robotContainer.Intake.setIntake();
    m_robotContainer.garra.setBraco();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
