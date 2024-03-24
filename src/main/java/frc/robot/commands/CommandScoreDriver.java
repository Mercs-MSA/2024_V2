package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.AmperSubcommands.CommandAmperScoreAmp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.amper.Amper;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class CommandScoreDriver extends Command {
    private Index m_index;
    private Shooter m_shooter;
    private AmperMotor m_amperMotor;

    public CommandScoreDriver(Shooter m_shooter, AmperMotor m_amperMotor, Index m_index) {
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

        m_index.startIndexMotor();
        
    }

    @Override
    public boolean isFinished(){
        return true;

    }

}
