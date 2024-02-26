package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.CustomGamePieceVision;

import javax.crypto.spec.RC2ParameterSpec;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandSwerveToNote extends Command {    
    private Swerve s_Swerve;
    private CustomGamePieceVision m_CustomGamePieceVision;

    public CommandSwerveToNote(Swerve s_Swerve, CustomGamePieceVision m_CustomGamePieceVision) {
        this.s_Swerve = s_Swerve;
        this.m_CustomGamePieceVision = m_CustomGamePieceVision;
        addRequirements(s_Swerve);
        addRequirements(m_CustomGamePieceVision);
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
            new Translation2d(0.0, 0.0), 
            0.0, 
            false, 
            false
        );
    }
  
    @Override
    public void execute() {
        /* Drive */
        // s_Swerve.drive(
        //     new Translation2d(0.0, m_CustomGamePieceVision.alignNoteCommands()[1]).times(Constants.Swerve.maxSpeed), 
        //     m_CustomGamePieceVision.alignNoteCommands()[0] * Constants.Swerve.maxAngularVelocity, 
        //     false, 
        //     true
        // );

        s_Swerve.driveToPose(new Pose2d(s_Swerve.poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d(Units.degreesToRadians(m_CustomGamePieceVision.getGamePieceYaw()))));
    }
  
    @Override
    public boolean isFinished() {
        // Should return true when no note is seen?
        //return Constants.isWithinTol(Constants.Vision.gamePieceYawOffset, m_CustomGamePieceVision.getGamePieceYaw(), 3);
        return false;
        
    }
}