// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ScoringConstants.ScoringMode;
import frc.robot.commands.CommandChangeScoringMode;
import frc.robot.commands.CommandScore;
import frc.robot.commands.CommandScoreDriver;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.AmperMotorSubcommands.CommandAmperMotorStopNeutral;
import frc.robot.commands.IndexSubcommands.CommandIndexReverse;
import frc.robot.commands.IndexSubcommands.CommandIndexStopNeutral;
import frc.robot.commands.IntakeSubcommands.CommandIntakeReverse;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStart;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStopNeutral;
import frc.robot.commands.PivotSubcommands.CommandPivotToPose;
import frc.robot.commands.ShooterSubcommands.CommandShooterStart;
import frc.robot.commands.ShooterSubcommands.CommandShooterStopNeutral;
import frc.robot.subsystems.amper.Amper;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.sensors.BeamBreak;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.ApriltagVision;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverJoystick = new CommandXboxController(0); 
  private final CommandXboxController operator = new CommandXboxController(1); 
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // TODO: THIS PROPORTIONAL GAIN VALUE IS PROBABLY TOO HIGH AND NEEDS TO BE TESTED/TUNED
  private final PhoenixPIDController turnPID = new PhoenixPIDController(3.2, 0.0, 0.0);                                                             

  /* Subsystems */
  public static final Intake m_intake = new Intake();
  public static final Index m_index = new Index();
  public static final Pivot m_pivot = new Pivot();
  public static final Shooter m_shooter = new Shooter();
  public static final Amper m_amper = new Amper();
  public static final AmperMotor m_amperMotor = new AmperMotor();
  public ApriltagVision m_ApriltagVision = new ApriltagVision("FR");
  public BeamBreak m_BeamBreak = new BeamBreak();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));


    // reset the field-centric heading on left bumper press
    driverJoystick.start().and(driverJoystick.back()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    driveAngle.HeadingController = turnPID;
    driveAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    driverJoystick.x().whileTrue(
        drivetrain.applyRequest(() -> {
            double currentYawInRadians = MathUtil.angleModulus(drivetrain.getState().Pose.getRotation().getRadians());
            
            // TODO: IF NO TAG IS SEEN, THIS TURNS INTO '-1' FOR SOME REASON, WHICH IS A BAD IDEA
            double radiansToTarget = MathUtil.angleModulus(Units.degreesToRadians(ApriltagVision.getYaw()));

            // TODO: IM NOT SURE IF THESE ANGLES SHOULD BE ADDED OR SUBTRACTED BASED ON WHETHER THEY'RE SIGNS ARE THE SAME DIRECTION
            Rotation2d desiredAngle = Rotation2d.fromRadians(currentYawInRadians + radiansToTarget);

            return driveAngle.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                .withTargetDirection(desiredAngle);
        }
    ));
  }

  public void driverControls(){
    driverJoystick.rightBumper().onTrue(
        new ParallelCommandGroup(
            new CommandScoreDriver(m_shooter, m_amperMotor, m_index),
            new CommandIntakeStart(m_intake)
        )
        
    )
    .onFalse(
        new SequentialCommandGroup(
            new CommandIntakeStopNeutral(m_intake),
            new CommandIndexStopNeutral(m_index),
            new CommandAmperMotorStopNeutral(m_amperMotor),
            new WaitCommand(0.1),
            new CommandShooterStopNeutral(m_shooter)
        )
    );

    driverJoystick.leftBumper().onTrue(
        // new CommandRotateToPose(s_Swerve, m_ApriltagVision)
        new InstantCommand(() -> Constants.Vision.autoAimActive = true)
    ).onFalse(
        new InstantCommand(() -> Constants.Vision.autoAimActive = false)
    );

    driverJoystick.b().onTrue(
        new InstantCommand(() -> Constants.Vision.manualDriveInvert = -1)
    );

    driverJoystick.a().onTrue(
        new InstantCommand(() -> Constants.Vision.manualDriveInvert = 1)
    );

}

public void operatorControls(){
  operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.PODIUM));
  operator.pov(90).onTrue(new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
  operator.pov(270).onTrue(new CommandChangeScoringMode(ScoringMode.AMP));
  operator.pov(180).onTrue(new CommandChangeScoringMode(ScoringMode.START));
  operator.a().whileTrue(
      new SequentialCommandGroup(
          new CommandChangeScoringMode(ScoringMode.AUTOAIM),
          new CommandPivotToPose(m_pivot, m_ApriltagVision).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      ));
  
  operator.leftBumper()
  .onTrue(
      new SequentialCommandGroup(
          // new InstantCommand(() -> m_BeamBreak.enableAsynchronousInterrupt()),
          // intakeNote(),
          // new CommandShooterReverse(m_shooter, 10, 10)
          new IntakeNote(m_intake, m_index, m_BeamBreak)
          // new CommandScore(m_amper, m_shooter, m_amperMotor)
      )
      
  )
  .onFalse(
      new SequentialCommandGroup(
          // new CommandShooterStopNeutral(m_shooter),
          stopIntakeIndexNeutral()
          // new ConditionalCommand(
          // new CommandShooterStart(m_shooter, -75, -55), 
          // getAutonomousCommand(), 
          // () -> m_BeamBreak.detectsNote())
      )
  );

  operator.rightBumper()
  .onTrue(
      expelNote()
  )
  .onFalse(
      stopIntakeIndexNeutral()
  );

  operator.x()
  .onTrue(
      new CommandPivotToPose(m_pivot, m_ApriltagVision).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
  );

  operator.y().onTrue(
      new SequentialCommandGroup(
          // new CommandIndexReverse(m_index),
          // new CommandShooterReverse(m_shooter),
          // new WaitCommand(0.1),
          // //new CommandShooterStopNeutral(m_shooter),
          // new CommandIndexStop(m_index),
          // new CommandShooterStopNeutral(m_shooter),
          new CommandScore(m_amper, m_shooter, m_amperMotor)
      )
  );

}

public Command stopIntakeIndexNeutral(){
  return new ParallelCommandGroup(
      new CommandIntakeStopNeutral(m_intake),
      new CommandIndexStopNeutral(m_index)
  );
}

public Command stopIntakeIndexShooterAmperNeutral(){
  return new ParallelCommandGroup(
      new CommandIntakeStopNeutral(m_intake),
      new CommandIndexStopNeutral(m_index),
      new CommandShooterStopNeutral(m_shooter),
      new CommandAmperMotorStopNeutral(m_amperMotor)
  );
}

public Command stopIndexShooterAmperNeutral(){
  return new ParallelCommandGroup(
      new CommandIndexStopNeutral(m_index),
      new CommandShooterStopNeutral(m_shooter),
      new CommandAmperMotorStopNeutral(m_amperMotor)
  );
}

public Command expelNote(){
  return new ParallelCommandGroup(
          new CommandIndexReverse(m_index),
          new CommandIntakeReverse(m_intake)
  );
}

public static Command stopEverything(){
  return new SequentialCommandGroup(
      // new WaitCommand(0.1),
      new CommandIntakeStopNeutral(m_intake),
      new CommandIndexStopNeutral(m_index),
      new CommandShooterStopNeutral(m_shooter),
      new CommandAmperMotorStopNeutral(m_amperMotor),
      new CommandShooterStart(m_shooter, -75, -55)
  );
}


  public RobotContainer() {
    configureBindings();
    driverControls();
    operatorControls();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
