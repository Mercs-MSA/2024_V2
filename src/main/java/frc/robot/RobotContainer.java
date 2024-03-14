package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandDriveToPose;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.ApriltagVision;
// import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.vision.CustomGamePieceVision;

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

    public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_yaw", "note_dist");
    public ApriltagVision m_ApriltagVision = new ApriltagVision();

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            
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
    }

    public void driverControls(){
        
    }

    public void operatorControls(){
        
    }

    // /**
    //  * Use this to pass the autonomous command to the main {@link Robot} class.
    //  *
    //  * @return the command to run in autonomous
    //  */
    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))),

            new CommandDriveToPose(s_Swerve, new Pose2d(2.85, 5.60, Rotation2d.fromDegrees(0))),

            new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub

            new CommandDriveToPose(s_Swerve, new Pose2d(2.67, 4.09, Rotation2d.fromDegrees(0))),

            new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))), //sub

            new CommandDriveToPose(s_Swerve, new Pose2d(2.77, 7.09, Rotation2d.fromDegrees(0))),

            new CommandDriveToPose(s_Swerve, new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))) //sub

        );
    }

}