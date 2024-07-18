// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreAmp;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreNote;
import frc.robot.commands.ShooterSubcommands.CommandShooterStart;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.pivot.Pivot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.sensors.BeamBreak;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // public static final CTREConfigs ctreConfigs = new CTREConfigs();
  
  
  private Command m_autonomousCommand;

  // private Pigeon2 pigeon2 = new Pigeon2(16, "canivore"); //change port


  private final RobotContainer m_robotContainer = new RobotContainer();
  private final CommandXboxController test_controller = new CommandXboxController(3); 


  Pose2d apiltagPlusGyro = new Pose2d();
  //private AnalogInput PSU_Volt_Monitor = new AnalogInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.


    Optional<Alliance> alliance = DriverStation.getAlliance();
    Constants.Vision.isRedAlliance = Constants.AllianceFlipUtil.shouldFlip();
    SmartDashboard.putBoolean("Are we red alliance?", Constants.Vision.isRedAlliance);

    SmartDashboard.putBoolean("Status 1", Pivot.status1OK);
    SmartDashboard.putBoolean("Status 2", Pivot.status2OK);

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

    LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    SmartDashboard.putNumber("poseX", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("poseY", m_robotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("test", new Rotation2d(m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() - (LimelightHelpers.getTX("limelight"))).getRadians());
    SmartDashboard.putNumber("limelight_ty", LimelightHelpers.getTY("limelight"));
    // SmartDashboard.putBoolean("hasNote", );

    SmartDashboard.putNumber("pivotRotationsTarget", Constants.Vision.pivotEncoderCalculator());

    SmartDashboard.putNumber("newPivotRotationsTarget", Constants.Vision.pivotAngleEquation());

    SmartDashboard.putNumber("xDistance", Constants.Vision.returnxDistance());

    SmartDashboard.putNumber("Interpolation", Constants.Vision.pivotInterpolationPosCalculator());

    
    // SmartDashboard.putNumber("hypotenuse1", Constants.Vision.returnHypotenuse1());

    // SmartDashboard.putNumber("xDistance", Constants.Vision.returnXdistance());

    // SmartDashboard.putNumber("AngleToApriltag", Constants.Vision.returnAngleToApriltag());

    // SmartDashboard.putNumber("hypotenuse2", Constants.Vision.returnHypotenuse2());

    SmartDashboard.putNumber("mod0CurrentDrive", m_robotContainer.drivetrain.getModule(0).getDriveMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod1CurrentDrive", m_robotContainer.drivetrain.getModule(1).getDriveMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod2CurrentDrive", m_robotContainer.drivetrain.getModule(2).getDriveMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod3CurrentDrive", m_robotContainer.drivetrain.getModule(3).getDriveMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod0CurrentSteer", m_robotContainer.drivetrain.getModule(0).getSteerMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod1CurrentSteer", m_robotContainer.drivetrain.getModule(1).getSteerMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod2CurrentSteer", m_robotContainer.drivetrain.getModule(2).getSteerMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod3CurrentSteer", m_robotContainer.drivetrain.getModule(3).getSteerMotor().getSupplyCurrent().getValue());
    SmartDashboard.putNumber("mod0Volt", m_robotContainer.drivetrain.getModule(0).getDriveMotor().getSupplyVoltage().getValue());
    SmartDashboard.putNumber("mod1Volt", m_robotContainer.drivetrain.getModule(1).getDriveMotor().getSupplyVoltage().getValue());
    SmartDashboard.putNumber("mod2Volt", m_robotContainer.drivetrain.getModule(2).getDriveMotor().getSupplyVoltage().getValue());
    SmartDashboard.putNumber("mod3Volt", m_robotContainer.drivetrain.getModule(3).getDriveMotor().getSupplyVoltage().getValue());
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_pivot.setBrakeMode(false);
  }
  
  @Override
  public void disabledPeriodic() {
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

    // m_robotContainer.s_Swerve.zeroGyro();
    
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
    m_robotContainer.stopIndexShooterAmperNeutral().schedule();
    m_robotContainer.m_pivot.setBrakeMode(true);
    Constants.Vision.autoRunning = false;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Constants.ScoringConstants.currentScoringMode == Constants.ScoringConstants.ScoringMode.AMP){
      new CommandAmperScoreAmp(m_robotContainer.m_amper).schedule();
      new CommandShooterStart(m_robotContainer.m_shooter, Constants.SATConstants.AMP.shooter1, Constants.SATConstants.AMP.shooter2).schedule();
    }
    else {
      new CommandAmperScoreNote(m_robotContainer.m_amper).schedule();
      new CommandShooterStart(m_robotContainer.m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter2).schedule();

    }

    
    var lastResult = LimelightHelpers.getLatestResults("limelight");
    if (lastResult.valid) {
    m_robotContainer.drivetrain.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue("limelight"), Timer.getFPGATimestamp());
      SmartDashboard.putBoolean("limelightResultValid", true);
    } else {
      SmartDashboard.putBoolean("limelightResultValid", false);
    }
  }

  private Joystick test_joystick = new Joystick(3);

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    test_controller.a().whileTrue(
      m_robotContainer.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
  );

  test_controller.b().whileTrue(
      m_robotContainer.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)
  );

  test_controller.x().whileTrue(
      m_robotContainer.drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)
  );

  test_controller.y().whileTrue(
      m_robotContainer.drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
  );
}
}