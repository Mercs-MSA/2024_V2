package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.vision.CustomGamePieceVision;
import frc.robot.subsystems.intake.Intake;

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

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    // public final SAT m_SAT = new SAT();
    public final Intake m_intake = new Intake();
    public final climber m_climber = new climber();
    //public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_pipeline");

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    /* Commands */
    public CommandOverrideIntakeStart commandOverrideIntakeStart = new CommandOverrideIntakeStart(m_intake);
    public CommandOverrideIndexStart commandOverrideIndexStart = new CommandOverrideIndexStart(m_intake);
    public CommandOverrideIntakeStop commandOverrideIntakeStop = new CommandOverrideIntakeStop(m_intake);
    public CommandOverrideIndexStop commandOverrideIndexStop = new CommandOverrideIndexStop(m_intake);

    // public CommandSwerveToHeading commandSwerveToHeading0 = new CommandSwerveToHeading(0, s_Swerve);
    // public CommandSwerveToHeading commandSwerveToHeading90 = new CommandSwerveToHeading(90, s_Swerve);
    // public CommandSwerveToHeading commandSwerveToHeading180 = new CommandSwerveToHeading(180, s_Swerve);
    // public CommandSwerveToHeading commandSwerveToHeading270 = new CommandSwerveToHeading(270, s_Swerve);

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);
    
    // public CommandBasesPosition commandGoToBasePodiumPosition = new CommandBasesPosition("Podium", m_SAT);
    // public CommandBasesPosition commandGoToBaseSubPosition = new CommandBasesPosition("Sub", m_SAT);
    // public CommandBasesPosition commandGoToBaseTrapPosition = new CommandBasesPosition("Trap", m_SAT);
    // public CommandBasesPosition commandGoToBaseZeroPosition = new CommandBasesPosition("Zero", m_SAT);
    // public CommandBasesPosition commandGoToBaseAmpPosition = new CommandBasesPosition("Amp", m_SAT);
    // public CommandBasesPosition commandGoToBaseWingPosition = new CommandBasesPosition("Wing", m_SAT);
    // public CommandPivotPosition commandGoToPivotPodiumPosition = new CommandPivotPosition("Podium", m_SAT);
    // public CommandPivotPosition commandGoToPivotSubPosition = new CommandPivotPosition("Sub", m_SAT);
    // public CommandPivotPosition commandGoToPivotTrapPosition = new CommandPivotPosition("Trap", m_SAT);
    // public CommandPivotPosition commandGoToPivotZeroPosition = new CommandPivotPosition("Zero", m_SAT);
    // public CommandPivotPosition commandGoToPivotAmpPosition = new CommandPivotPosition("Amp", m_SAT);
    // public CommandPivotPosition commandGoToPivotWingPosition = new CommandPivotPosition("Wing", m_SAT);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        // Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Finished 1 Piece"));
        NamedCommands.registerCommand("marker2", Commands.print("Finished 3-4 Piece"));

        NamedCommands.registerCommand("Start Intake", m_intake.collectNote());
        NamedCommands.registerCommand("Intake To Index", m_intake.passNoteToIndex());

        // NamedCommands.registerCommand("Go To Base Podium Positon", Commands.runOnce(() -> m_SAT.goToBasePodiumPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base AMP Positon", Commands.runOnce(() -> m_SAT.goToBaseAmpPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Sub Positon", Commands.runOnce(() -> m_SAT.goToBaseSubPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Trap Positon", Commands.runOnce(() -> m_SAT.goToBaseTrapPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Zero Positon", Commands.runOnce(() -> m_SAT.goToBaseZeroPosition(), m_SAT));

        // NamedCommands.registerCommand("Go To Pivot Podium Positon", Commands.runOnce(() -> m_SAT.goToPivotPodiumPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot AMP Positon", Commands.runOnce(() -> m_SAT.goToPivotAmpPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Sub Positon", Commands.runOnce(() -> m_SAT.goToPivotSubPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Trap Positon", Commands.runOnce(() -> m_SAT.goToPivotTrapPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Zero Positon", Commands.runOnce(() -> m_SAT.goToPivotZeroPosition(), m_SAT));
         
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void configureButtonBindings() {
        /************************/
        /*                      */
        /*    Driver Buttons    */
        /*                      */
        /************************/
        
        // driver.y()
        //    .onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));
         
        // driver.povUp()
        //     .onTrue(commandSwerveToHeading0);
        // driver.povLeft()
        //     .onTrue(commandSwerveToHeading90);
        // driver.povDown()
        //     .onTrue(commandSwerveToHeading180);
        // driver.povRight()
        //     .onTrue(commandSwerveToHeading270);

        // driver.a()
        //     .onTrue(commandSwerveToNote.alongWith(m_intake.collectNote()));

        // driver.leftTrigger()
        //     .onTrue(Commands.runOnce(() -> m_SAT.shootNote(), m_SAT));
        // driver.rightTrigger()
        //     .onTrue(Commands.runOnce(() -> m_SAT.stopShooter(), m_SAT));


        /************************/
        /*                      */
        /*   Operator Buttons   */
        /*                      */
        /************************/

        operator.axisGreaterThan(5, 0.5)
            .whileTrue(
                m_climber.climbDownCommand()
            );

        operator.axisLessThan(5, -0.5)
            .whileTrue(
                m_climber.climbUpCommand()
            );

        operator.axisLessThan(5, 0.5)
            .whileTrue(
                m_climber.climbMotorStop()
            );

        operator.axisGreaterThan(5, -0.5)
            .whileTrue(
                m_climber.climbMotorStop()
            );
            
        operator.axisGreaterThan(1, 0.5)
            .whileTrue(
                m_climber.climbMidLeftCommand()
            );

        operator.axisLessThan(1, -0.5)
            .whileTrue(
                m_climber.climbMidRightCommand()
            );

        operator.y()
            .whileTrue(
                m_climber.climbDownRightCommand()
            );
    
        operator.x()
            .whileTrue(
                m_climber.climbUpRightCommand()
            );
            
        operator.a()
            .whileTrue(
                m_climber.climbUpLeftCommand()
            );
    
        operator.b()
            .whileTrue(
                m_climber.climbDownLeftCommand()
            );
        // operator.x()
        //     .onTrue(
        //         Commands.runOnce(() -> m_SAT.goToPivotAmpPosition(), m_SAT)
        //         .andThen(Commands.runOnce(() -> m_SAT.goToBaseAmpPosition(), m_SAT))
        //     );
        // operator.a()
        //     .onTrue(commandGoToBaseZeroPosition.andThen(commandGoToPivotZeroPosition));
        
        // operator.b()
        //     .and(operator.axisGreaterThan(1, 0.6))
        //     .and(operator.axisLessThan(0, 0.4))
        //     .and(operator.axisGreaterThan(0, -0.4))
        //     .onTrue(commandGoToBasePodiumPosition.andThen(commandGoToPivotPodiumPosition));

        // operator.b()
        //     .and(operator.axisLessThan(1, -0.6))
        //     .and(operator.axisLessThan(0, 0.4))
        //     .and(operator.axisGreaterThan(0, -0.4))
        //     .onTrue(commandGoToBaseSubPosition.andThen(commandGoToPivotSubPosition));
        
        // operator.b()
        //     .and(operator.axisLessThan(0, -0.6))
        //     .and(operator.axisLessThan(1, 0.4))
        //     .and(operator.axisGreaterThan(1, -0.4))
        //     .onTrue(commandGoToBaseTrapPosition.andThen(commandGoToPivotTrapPosition));

        // operator.b()
        //     .and(operator.axisGreaterThan(0, 0.6))
        //     .and(operator.axisLessThan(1, 0.4))
        //     .and(operator.axisGreaterThan(1, -0.4))
        //     .onTrue(commandGoToBaseAmpPosition.andThen(commandGoToPivotAmpPosition));

        // operator.start()
        //     .onTrue(m_intake.collectNote());

        // operator.pov(0)
        //     .onTrue(Commands.run(() -> m_SAT.baseGoToPosition(0.05), m_SAT));

        // operator.pov(180)
        //     .onTrue(Commands.run(() -> m_SAT.baseGoToPosition(-0.05), m_SAT));

        // operator.leftBumper()
        //     .onTrue(Commands.run(() -> m_SAT.shootNote(true), m_SAT));

        // operator.rightBumper()
        //     .onTrue(Commands.run(() -> m_SAT.shootNote(false), m_SAT));
        
        operator.start()
            .whileTrue(commandOverrideIntakeStart.andThen(commandOverrideIndexStart));

        operator.start()
            .whileFalse(commandOverrideIntakeStop.andThen(commandOverrideIndexStop));
        
        if (m_intake.simulationDebugMode) {
            operator.a()
                .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(true);}));
            operator.b()
                .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(true);}));
            operator.x()
                .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(false);}));
            operator.y()
                .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(false);}));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}