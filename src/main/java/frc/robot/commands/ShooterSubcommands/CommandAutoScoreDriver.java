package frc.robot.commands.ShooterSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.CommandScoreDriver;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.shooter.Shooter;

public class CommandAutoScoreDriver extends Command {
    private Index m_index;
    private Shooter m_shooter;
    private AmperMotor m_amperMotor;

    public CommandAutoScoreDriver(Shooter m_shooter, AmperMotor m_amperMotor, Index m_index) {
        this.m_index = m_index;
        this.m_shooter = m_shooter;
        this.m_amperMotor = m_amperMotor;
    }

    @Override
    public void initialize() {

        if (ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AMP){
            m_amperMotor.startAmperMotor();
            m_shooter.setBothShooterMotor(SATConstants.AMP.shooter1, SATConstants.AMP.shooter2);
            
        }
        else {
            m_amperMotor.stopAmperMotor();
            
        }



        

        
        
    }

    @Override
    public void execute() {

        if ((Math.abs(LimelightHelpers.getTX("limelight")) < 10)) {
            m_index.startIndexMotor();
    }

        else { 
            m_index.stopIndexMotor();
        }
    }

    @Override
    public boolean isFinished(){
        return false;

    }

}