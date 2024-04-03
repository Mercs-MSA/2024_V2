// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreAmp;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreNote;
import frc.robot.commands.ShooterSubcommands.CommandShooterStart;
import frc.robot.subsystems.vision.ApriltagVision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean UseLimelight = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /**
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    // if (UseLimelight) {
    //   var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;

    //   Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

    //   if (lastResult.valid) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
    //   }
    // }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    Constants.Vision.visionTurnedOn = false;
    m_robotContainer.m_pivot.setBrakeMode(true);
    Constants.Vision.autoRunning = true;
    m_robotContainer.m_BeamBreak.disableAsynchronousInterrupt();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Constants.ScoringConstants.currentScoringMode =Constants.ScoringConstants.ScoringMode.INTAKE;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    
    Constants.Vision.visionTurnedOn = true;

    new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
    m_robotContainer.stopIndexShooterAmperNeutral();
    m_robotContainer.m_pivot.setBrakeMode(true);
    Constants.Vision.autoRunning = false;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("current heading", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("desiredAngle", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() - ApriltagVision.getYaw());

    if (Constants.ScoringConstants.currentScoringMode == Constants.ScoringConstants.ScoringMode.AMP){
      new CommandAmperScoreAmp(m_robotContainer.m_amper).schedule();
      // new CommandShooterStart(m_robotContainer.m_shooter, Constants.SATConstants.AMP.shooter1, Constants.SATConstants.AMP.shooter2).schedule();
    }
    else if (Constants.ScoringConstants.currentScoringMode == Constants.ScoringConstants.ScoringMode.INTAKE){
      //nothing
    }
    else {
      new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
      // new CommandShooterStart(m_robotContainer.m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter2).schedule();
    }

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
