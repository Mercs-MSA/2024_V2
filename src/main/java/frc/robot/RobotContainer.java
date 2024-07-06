// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import frc.robot.commands.CommandRotateToPose;
import frc.robot.commands.CommandScore;
import frc.robot.commands.CommandScoreDriver;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.AmperMotorSubcommands.CommandAmperMotorStopNeutral;
import frc.robot.commands.IndexSubcommands.CommandIndexReverse;
import frc.robot.commands.IndexSubcommands.CommandIndexStart;
import frc.robot.commands.IndexSubcommands.CommandIndexStartAuto;
import frc.robot.commands.IndexSubcommands.CommandIndexStop;
import frc.robot.commands.IndexSubcommands.CommandIndexStopNeutral;
import frc.robot.commands.IntakeSubcommands.CommandIntakeReverse;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStart;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStop;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStopNeutral;
import frc.robot.commands.PivotSubcommands.CommandAutoPivotAim;
import frc.robot.commands.PivotSubcommands.CommandPivotToPose;
import frc.robot.commands.ShooterSubcommands.CommandShooterStart;
import frc.robot.commands.ShooterSubcommands.CommandShooterStopNeutral;
import frc.robot.commands.IntakeNoteTele;
import frc.robot.commands.PivotSubcommands.CommandPivotShunt;
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

   public ProfiledPIDController thetaController =
      new ProfiledPIDController(10, 0.01, 0.1, new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

  private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
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
  public ApriltagVision m_ApriltagVision = new ApriltagVision("FL");
  public BeamBreak m_BeamBreak = new BeamBreak();

  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("Tests");

  /* AutoChooser */
  private final SendableChooser<Command> autoChooser;

  /* NamedCommands */
  Map<String, Command> autonomousCommands = new HashMap<String, Command>() {
    {
        /* Single Commands Each Subsystem */
        put("Start Intake", new CommandIntakeStart(m_intake));
        put("Start Index", new CommandIndexStart(m_index));
        put("Start Shooter", new CommandShooterStart(m_shooter, -60, -45));

        /* Reset Commands */
        put("Reset All", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, Constants.SATConstants.START.pivot), 
            // new CommandShooterStart(m_shooter, 0, 0), 
            new CommandShooterStopNeutral(m_shooter),
            new CommandIntakeStop(m_intake), 
            new CommandIndexStop(m_index)
        ));

        put("Reset SAT", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, Constants.SATConstants.START.pivot), 
            new CommandShooterStart(m_shooter, 0, 0)
        ));

        put("Reset II", new SequentialCommandGroup(
            new CommandIndexReverse(m_index), 
            new CommandIntakeReverse(m_intake), 
            new WaitCommand(0.01),
            new CommandIntakeStop(m_intake),
            new CommandIndexStop(m_index)
        ));

        put("Stop Intake", new CommandIntakeStop(m_intake));
        put("Stop Index", new CommandIndexStop(m_index));
        put("Stop Shooter", new CommandShooterStart(m_shooter, 0, 0));
        put("Reset Pivot", new CommandPivotToPose(m_pivot, Constants.SATConstants.START.pivot));

        /* Pivot Positions */

        put("Podium Pivot", new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.PODIUM),
            new CommandPivotToPose(m_pivot), 
            new CommandShooterStart(m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter1)
        ));
        
        put("Sub Pivot", new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            new CommandPivotToPose(m_pivot), 
            new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter1)
        ));

        put("Wing Pivot", new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.WING),
            new CommandPivotToPose(m_pivot), 
            new CommandShooterStart(m_shooter, Constants.SATConstants.WING.shooter1, Constants.SATConstants.WING.shooter1)
        ));

        //Pivot Positions for CENTER
        put("Podium Pivot Center", new CommandPivotToPose(m_pivot, 30)); // RED: 41.5  BLUE: 43 JUST CHNAGED FROM 43
        put("Center Pivot", new CommandPivotToPose(m_pivot, 30)); // RED: 44  BLUE: 46 JUST CHANGED FROM 46
        put("AMP Pivot", new CommandPivotToPose(m_pivot, 25.2)); // RED: 39  BLUE: 41 JUST CHANGED FROM 41

        put("Auto Pivot", 
        new SequentialCommandGroup(
          new CommandChangeScoringMode(ScoringMode.AUTOAIM),
          new CommandPivotToPose(m_pivot, m_ApriltagVision).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      ));

        put("Sub Auto Pivot", 
          new SequentialCommandGroup(
          new CommandChangeScoringMode(ScoringMode.AUTOAIM),
          new CommandPivotToPose(m_pivot, m_ApriltagVision).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
          new CommandShooterStart(m_shooter, Constants.SATConstants.WING.shooter1, Constants.SATConstants.WING.shooter1)
      ));
                  


        //Pivot Positions for AMPSIDE
        put("Amp Pivot Preload", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, 51), 
            new CommandShooterStart(m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter1)
        ));

        put("Amp Pivot Wing", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, 51), 
            new CommandShooterStart(m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter1)
        ));

        put("Amp Pivot Line", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, 32), 
            new CommandShooterStart(m_shooter, -70, -50)
        ));

        //Pivot Positions for SOURCESIDE
        put("Source Pivot Preload", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, 36), 
            new CommandShooterStart(m_shooter, -75, -55)
        ));

        put("Source Pivot Line", new ParallelCommandGroup(
            new CommandPivotToPose(m_pivot, 31), 
            new CommandShooterStart(m_shooter, -75, -55)
        ));

        put("Start Shooter Poop", new CommandShooterStart(m_shooter, -20, -20));

        /* Intake */
        put("Intake Note", new IntakeNote(m_intake, m_index, m_BeamBreak));
        put("Intake Noteyyyyyyg", new ParallelCommandGroup(
            new WaitCommand(0.2),
            // new CommandIntakeStart(m_intake) 
            new CommandIndexStart(m_index)
        ));

        /* Score Note */
        put("Score", new SequentialCommandGroup(
            new WaitCommand(0.2), 
            new CommandIndexStart(m_index)
        ));

        put("Score Auto", new SequentialCommandGroup(
            new WaitCommand(0.2), 
            new CommandIndexStartAuto(m_index, m_BeamBreak)
        ));

        put("Auto Pivot", new CommandAutoPivotAim(m_pivot));
    }
  };

  private final Telemetry logger = new Telemetry(MaxSpeed);

  double limelight_aim_proportional() {
    double kP = 0.020;
    int[] ids = new int[2];
    ids[0] = 7;
    ids[1] = 4;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", ids);

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    targetingAngularVelocity *= MaxAngularRate;
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
}
  public void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(Constants.Vision.manualDriveInvert * -driverJoystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(Constants.Vision.manualDriveInvert * -driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(false));
        driverJoystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> 
          drive.withVelocityX(driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(limelight_aim_proportional())
        ));


    // reset the field-centric heading on left bumper press
    driverJoystick.start().and(driverJoystick.back()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    driveAngle.HeadingController = new PhoenixPIDController(12.5, 0.0, 0.0);  ;
    driveAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

   // driverJoystick.leftBumper().onTrue(drivetrain.applyRequest(() -> driveAngle.withTargetDirection(new Rotation2d(drivetrain.getState().Pose.getRotation().getDegrees() - LimelightHelpers.getTX("limelight")))));
  }

  public void driverControls(){
    driverJoystick.rightBumper().onTrue(
        new ParallelCommandGroup(
            new CommandScoreDriver(m_shooter, m_amperMotor, m_index)
            //new CommandIntakeStart(m_intake)
        )
        
    )
    .onFalse(
        new SequentialCommandGroup(
            new CommandIntakeStopNeutral(m_intake),
            new CommandIndexStopNeutral(m_index),
            new CommandAmperMotorStopNeutral(m_amperMotor)
            // new WaitCommand(0.1),
            // new CommandShooterStopNeutral(m_shooter)
        )
    );

    // driverJoystick.leftBumper().onTrue(
    //     // new CommandRotateToPose(s_Swerve, m_ApriltagVision)
    //     new InstantCommand(() -> Constants.Vision.autoAimActive = true)
    // ).onFalse(
    //     new InstantCommand(() -> Constants.Vision.autoAimActive = false)
    // );

    driverJoystick.b().onTrue(
        new InstantCommand(() -> Constants.Vision.manualDriveInvert = -1)
    );

    driverJoystick.a().onTrue(
        new InstantCommand(() -> Constants.Vision.manualDriveInvert = 1)
    );

}


public void operatorControls(){
  operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.AUTOAIM));
  operator.pov(90).onTrue(new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
  operator.pov(270).onTrue(new CommandChangeScoringMode(ScoringMode.AMP));
  operator.pov(180).onTrue(new CommandChangeScoringMode(ScoringMode.START));
  operator.a().whileTrue(
      new SequentialCommandGroup(
          new CommandChangeScoringMode(ScoringMode.AUTOAIM),
          new CommandPivotToPose(m_pivot, m_ApriltagVision).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
      ));

  operator.leftBumper()
  .whileTrue(
      new SequentialCommandGroup(
          // new InstantCommand(() -> m_BeamBreak.enableAsynchronousInterrupt()),
          // intakeNote(),
          // new CommandShooterReverse(m_shooter, 10, 10)
          new IntakeNoteTele(m_intake, m_index, m_BeamBreak)
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

  operator.y()
  .onTrue(
      new CommandPivotShunt(m_pivot).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
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

  operator.start().and(operator.back()).onTrue(
    new InstantCommand(() -> m_pivot.applyConfig())
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
  return new SequentialCommandGroup(
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

    NamedCommands.registerCommands(autonomousCommands);

    autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }
}
