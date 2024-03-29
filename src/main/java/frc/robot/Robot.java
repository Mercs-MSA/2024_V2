// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreAmp;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreNote;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer = new RobotContainer();

  Pose2d apiltagPlusGyro = new Pose2d();
  private AnalogInput PSU_Volt_Monitor = new AnalogInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer.configureButtonBindings();

    Optional<Alliance> alliance = DriverStation.getAlliance();
    Constants.Vision.isRedAlliance = Constants.AllianceFlipUtil.shouldFlip();
    SmartDashboard.putBoolean("Are we red alliance?", Constants.Vision.isRedAlliance);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("robot state", Constants.ScoringConstants.currentScoringMode.toString());

    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.m_pivot.setBrakeMode(false);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // if (Constants.AllianceFlipUtil.shouldFlip()){
    //   m_robotContainer.s_Swerve.gyro.setYaw(180);
    // }
    // else {
    //   m_robotContainer.s_Swerve.gyro.setYaw(0);
    // }
    m_robotContainer.m_BeamBreak.disableAsynchronousInterrupt();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.s_Swerve.zeroGyro();
    
    new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  
    Constants.Vision.visionTurnedOn = false;
    m_robotContainer.m_pivot.setBrakeMode(true);

    Constants.Vision.autoRunning = true;

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Constants.Vision.visionTurnedOn = true;

    new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
    m_robotContainer.stopIndexShooterAmperNeutral();
    m_robotContainer.m_pivot.setBrakeMode(true);
    Constants.Vision.autoRunning = false;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Constants.ScoringConstants.currentScoringMode == Constants.ScoringConstants.ScoringMode.AMP){
      new CommandAmperScoreAmp(m_robotContainer.m_amper).schedule();
    }
    else {
      new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

}