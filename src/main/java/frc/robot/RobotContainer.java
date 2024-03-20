package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ScoringMode;
import frc.robot.commands.CommandChangeScoringMode;
import frc.robot.commands.CommandDriveToPose;
import frc.robot.commands.CommandRotateToPose;
import frc.robot.commands.CommandScoreDriver;
import frc.robot.commands.CommandScoreOperator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.AmperMotorSubcommands.CommandAmperMotorReverse;
import frc.robot.commands.AmperMotorSubcommands.CommandAmperMotorStart;
import frc.robot.commands.AmperMotorSubcommands.CommandAmperMotorStopNeutral;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreAmp;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreNote;
import frc.robot.commands.IndexSubcommands.CommandIndexReverse;
import frc.robot.commands.IndexSubcommands.CommandIndexStart;
import frc.robot.commands.IndexSubcommands.CommandIndexStop;
import frc.robot.commands.IndexSubcommands.CommandIndexStopNeutral;
import frc.robot.commands.IntakeSubcommands.CommandIntakeReverse;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStart;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStop;
import frc.robot.commands.IntakeSubcommands.CommandIntakeStopNeutral;
import frc.robot.commands.PivotSubcommands.CommandPivotDynamicPose;
import frc.robot.commands.PivotSubcommands.CommandPivotToNeutral;
import frc.robot.commands.PivotSubcommands.CommandPivotToPose;
import frc.robot.commands.ShooterSubcommands.CommandShooterReverse;
import frc.robot.commands.ShooterSubcommands.CommandShooterStart;
import frc.robot.commands.ShooterSubcommands.CommandShooterStopNeutral;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.amper.Amper;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.ApriltagVision;
// import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.vision.CustomGamePieceVision;

import edu.wpi.first.hal.DriverStationJNI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandXboxController tester = new CommandXboxController(2);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Intake m_intake = new Intake();
    public static final Index m_index = new Index();
    public static final Pivot m_pivot = new Pivot();
    public static final Shooter m_shooter = new Shooter();
    public static final Amper m_amper = new Amper();
    public static final AmperMotor m_amperMotor = new AmperMotor();

    public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_yaw", "note_dist");
    public ApriltagVision m_ApriltagVision = new ApriltagVision();

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("Start Intake", new CommandIntakeStart(m_intake));
            put("Start Index", new CommandIndexStart(m_index));
    
            put("Reset", new ParallelCommandGroup(
                new CommandPivotToPose(m_pivot, Constants.SATConstants.START.pivot),
                new CommandShooterStart(m_shooter, 0, 0)
            ));

            put("Podium Pivot", new SequentialCommandGroup(
                new CommandPivotToPose(m_pivot, 40), 
                new CommandShooterStart(m_shooter, Constants.SATConstants.PODIUM.shooter1, Constants.SATConstants.PODIUM.shooter2)
            ));

            put("Move Pivot Lower 1", new CommandPivotToPose(m_pivot, 40));
            put("Move Pivot Lower 2", new CommandPivotToPose(m_pivot, 37));

            put("Intake Note", new SequentialCommandGroup(
                new CommandIntakeStart(m_intake),
                new CommandIndexStart(m_index)
            ));

            put("score sub note", new SequentialCommandGroup(
                new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
                new ParallelCommandGroup(
                    new CommandPivotToPose(m_pivot, Constants.SATConstants.SUB.pivot),
                    new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter2)
                ),
                new WaitCommand(.2),
                new CommandIndexStart(m_index)
            ));

            put("Reverse", new SequentialCommandGroup(
                new CommandIndexReverse(m_index),
                new WaitCommand(0.1),
                new CommandIntakeReverse(m_intake))
                );

            put("score podium note", new SequentialCommandGroup(
                new CommandChangeScoringMode(ScoringMode.PODIUM),
                new ParallelCommandGroup(
                    new CommandIndexStop(m_index),
                    new CommandIntakeStop(m_intake)
                ),
                new WaitCommand(.3),
                new CommandIndexStart(m_index)
            ));

            put("score wing note", new SequentialCommandGroup(
                new CommandChangeScoringMode(ScoringMode.WING),
                new ParallelCommandGroup(
                    new CommandPivotToPose(m_pivot, Constants.SATConstants.WING.pivot),
                    new CommandShooterStart(m_shooter, Constants.SATConstants.WING.shooter1, Constants.SATConstants.WING.shooter2)
                ),
                new WaitCommand(.5),
                new CommandIndexStart(m_index)
            ));
        }  
    };

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> false // just hardcoded field centric... could make this a button if we want
            )
        );

        NamedCommands.registerCommands(autonomousCommands);

        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void configureButtonBindings() {
        driverControls();
        operatorControls();
        // manualTesting();

    }

    public void driverControls(){
        driver.rightBumper().onTrue(
            new SequentialCommandGroup(
                new CommandIndexReverse(m_index),
                new CommandShooterReverse(m_shooter),
                new WaitCommand(0.1),
                new CommandShooterStopNeutral(m_shooter),
                new CommandIndexStop(m_index),
                new CommandScoreDriver(m_amper, m_shooter, m_amperMotor),
                new CommandIndexStart(m_index)
            )
        )
        .onFalse(
            stopIntakeIndexNeutral()
  
        );

        driver.leftBumper().onTrue(
            new CommandRotateToPose(s_Swerve, new Pose2d(Swerve.poseEstimator.getEstimatedPosition().getX(), Swerve.poseEstimator.getEstimatedPosition().getY(), Rotation2d.fromDegrees(-23.9)))
        );
        // driver.getRightTriggerAxis().greaterTh(
        //    new CommandScoreDriver(m_pivot, m_amper, m_index, m_shooter, m_amperMotor); 
        // );
    }

    public void manualTesting(){
        operator.pov(0).whileTrue(new RunCommand(() -> m_pivot.leaderGoToPositionIncrement(0.25), m_pivot));
        operator.pov(180).whileTrue(new RunCommand(() -> m_pivot.leaderGoToPositionIncrement(-0.25), m_pivot));

                
        operator.leftBumper()
        .onTrue(
            new ParallelCommandGroup(
                intakeNote(),
                new CommandShooterReverse(m_shooter, 10, 10)
            )
            
        )
        .onFalse(
            new SequentialCommandGroup(
                // new CommandShooterStopNeutral(m_shooter),
                // new CommandIndexReverse(m_index),
                // new WaitCommand(0.1),
                stopIntakeIndexNeutral()
            )
        );

        operator.rightBumper()
        .onTrue(
            expelNote()
        )
        .onFalse(
            stopIntakeIndexNeutral()
        );

        operator.a().onTrue(
            new CommandAmperScoreNote(m_amper)
        );


        operator.y().onFalse(
            new CommandAmperScoreAmp(m_amper)
        );

        operator.start().onTrue(
            new CommandPivotDynamicPose(m_pivot)
        );
        // operator.a().onTrue(
        //     new CommandAmperMotorReverse(m_amperMotor)
        // )
        // .onFalse(
        //     new CommandAmperMotorStopNeutral(m_amperMotor)
        // );

        // operator.y().onTrue(
        //     new CommandAmperMotorStart(m_amperMotor)
        // )
        // .onFalse(
        //     new CommandAmperMotorStopNeutral(m_amperMotor)
        // );
    }

    public void operatorControls(){
        operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.PODIUM));
        operator.pov(90).onTrue(new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
        operator.pov(180).onTrue(new CommandChangeScoringMode(ScoringMode.WING));
        operator.pov(270).onTrue(new CommandChangeScoringMode(ScoringMode.AMP));
        //don't do START position, use a on operator
        
        operator.leftBumper()
        .onTrue(
            new ParallelCommandGroup(
                intakeNote(),
                new CommandShooterReverse(m_shooter, 10, 10)
            )
            
        )
        .onFalse(
            new SequentialCommandGroup(
                new CommandShooterStopNeutral(m_shooter),
                new CommandIndexReverse(m_index),
                new WaitCommand(0.1),
                stopIntakeIndexNeutral()
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
            new CommandScoreOperator(m_pivot, m_amper, m_index, m_shooter, m_amperMotor)
        )
        .onFalse(
            stopIntakeIndexShooterAmperNeutral()
        );

        operator.a().onTrue(
            new CommandPivotToPose(m_pivot, 0.2)

        );

        operator.y().onTrue(
            shootNote(0, -35)
        );

    }

    public Command intakeNote(){
        return new ParallelCommandGroup(
                new CommandIntakeStart(m_intake),
                new CommandIndexStart(m_index)
        );
        
    }

    public Command intakeNoteAuto(){
        return new ParallelCommandGroup(
                new CommandIntakeStart(m_intake),
                new CommandIndexStart(m_index),
                new CommandShooterReverse(m_shooter)
        );
        
    }

    public Command expelNote(){
        return new ParallelCommandGroup(
                new CommandIndexReverse(m_index),
                new CommandIntakeReverse(m_intake)
        );
    }

    public Command goToStart(){
        return new CommandPivotToPose(m_pivot, Constants.SATConstants.START.pivot);
    }

    public void periodic(){

    }

    public Command scoreNote(){
        double pivotPos = 0.0;
        double shooterMotorSpeed1 = 0.0;
        double shooterMotorSpeed2 = 0.0;
        SmartDashboard.putNumber("pivotPosVariable", pivotPos);
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                pivotPos = SATConstants.PODIUM.pivot;
                shooterMotorSpeed1 = SATConstants.PODIUM.shooter1;
                shooterMotorSpeed2 = SATConstants.PODIUM.shooter2;
                break;
            case SUBWOOFER:
                pivotPos = SATConstants.SUB.pivot;
                shooterMotorSpeed1 = SATConstants.PODIUM.shooter1;
                shooterMotorSpeed2 = SATConstants.PODIUM.shooter2;
                break;
            case WING:
                pivotPos = SATConstants.WING.pivot;
                shooterMotorSpeed1 = SATConstants.PODIUM.shooter1;
                shooterMotorSpeed2 = SATConstants.PODIUM.shooter2;
                break;
            case AMP:
                pivotPos = SATConstants.AMP.pivot;
                shooterMotorSpeed1 = SATConstants.AMP.shooter1;
                shooterMotorSpeed2 = SATConstants.AMP.shooter2;
                break;
            case START:
                pivotPos = SATConstants.START.pivot;
                shooterMotorSpeed1 = SATConstants.START.shooter1;
                shooterMotorSpeed2 = SATConstants.START.shooter2;
                break;
            default:
                pivotPos = SATConstants.SUB.pivot;
                shooterMotorSpeed1 = SATConstants.SUB.shooter1;
                shooterMotorSpeed2 = SATConstants.SUB.shooter2;
                break;
        }

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ConditionalCommand(
                    new CommandPivotToPose(m_pivot, Constants.Vision.pivotEncoderCalculator(Swerve.poseEstimator.getEstimatedPosition())),
                    new CommandPivotToPose(m_pivot, pivotPos), 
                    () -> ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AUTOAIM
                ),
                new ConditionalCommand(
                    new ParallelCommandGroup(
                        new CommandAmperScoreAmp(m_amper),
                        new CommandAmperMotorStart(m_amperMotor)
                    ),   
                    new CommandAmperScoreNote(m_amper), 
                    () -> ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AMP)
            ),
            shootNote(shooterMotorSpeed1, shooterMotorSpeed2)
            // new WaitCommand(1),
            
        );
    }

    public Command shootNote(double shooterMotorSpeed1, double shooterMotorSpeed2){
        return new SequentialCommandGroup(
            new CommandShooterStart(m_shooter, shooterMotorSpeed1, shooterMotorSpeed2),
            intakeNote()
        );
    }

    public Command shootNote(){
        return shootNote(-75, -60);
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
            new PrintCommand("it ran stopIntakeIndexShooterAmperNeutral"),
            new CommandIndexStopNeutral(m_index),
            new CommandShooterStopNeutral(m_shooter),
            new CommandAmperMotorStopNeutral(m_amperMotor)
        );
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected(); 
    }

    // /**
    //  * Use this to pass the autonomous command to the main {@link Robot} class.
    //  *
    //  * @return the command to run in autonomous
    //  */
    // public Command getAutonomousCommand() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))),

    //         new CommandPivotToPose(m_pivot, Constants.SATConstants.SUB.pivot),
    //         new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter2),
    //         new CommandIndexStart(m_index),
    //         new WaitCommand(0.2),
    //         new CommandShooterStopNeutral(m_shooter),


    //         new ParallelCommandGroup(
    //             intakeNoteAuto(),
    //             new CommandDriveToPose(s_Swerve, new Pose2d(2.85, 5.60, Rotation2d.fromDegrees(0)))
    //         ),
    //         new WaitCommand(0.2),
    //         new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub
    //         new CommandIndexReverse(m_index),
    //         new CommandShooterReverse(m_shooter),
    //         new WaitCommand(0.1),
    //         new CommandShooterStopNeutral(m_shooter),
    //         new CommandIndexStop(m_index),
    //         new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter2),
    //         new CommandIndexStart(m_index),
    //         new WaitCommand(0.2),
    //         new CommandShooterStopNeutral(m_shooter),

            
    //         new ParallelCommandGroup(
    //             intakeNoteAuto(),
    //             new CommandDriveToPose(s_Swerve, new Pose2d(2.67, 4.09, Rotation2d.fromDegrees(0)))
    //         ),
    //         new WaitCommand(0.2),
    //         new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub
    //         new CommandIndexReverse(m_index),
    //         new CommandShooterReverse(m_shooter),
    //         new WaitCommand(0.1),
    //         new CommandShooterStopNeutral(m_shooter),
    //         new CommandIndexStop(m_index),
    //         new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter2),
    //         new CommandIndexStart(m_index),
    //         new WaitCommand(0.2),
    //         new CommandShooterStopNeutral(m_shooter),

    //         new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub

    //         new ParallelCommandGroup(
    //             intakeNoteAuto(),
    //             new CommandDriveToPose(s_Swerve, new Pose2d(2.77, 7.09, Rotation2d.fromDegrees(0)))
    //         ),
    //         new WaitCommand(0.2),
    //         new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub
    //         new CommandIndexReverse(m_index),
    //         new CommandShooterReverse(m_shooter),
    //         new WaitCommand(0.1),
    //         new CommandShooterStopNeutral(m_shooter),
    //         new CommandIndexStop(m_index),
    //         new CommandShooterStart(m_shooter, Constants.SATConstants.SUB.shooter1, Constants.SATConstants.SUB.shooter2),
    //         new CommandIndexStart(m_index),
    //         new WaitCommand(0.2),
    //         new CommandShooterStopNeutral(m_shooter)

    //     );
    // }

}