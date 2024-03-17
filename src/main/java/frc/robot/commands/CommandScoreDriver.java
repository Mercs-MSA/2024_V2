package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.amper.Amper;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class CommandScoreDriver extends Command {
    private double shooterMotorSpeed1, shooterMotorSpeed2 = 0;
    private Pivot m_pivot;
    private Amper m_amper;
    private Index m_index;
    private Shooter m_shooter;
    private AmperMotor m_amperMotor;

    public CommandScoreDriver(Amper m_amper, Shooter m_shooter, AmperMotor m_amperMotor) {
        this.m_amper = m_amper;
        this.m_index = m_index;
        this.m_shooter = m_shooter;
        this.m_amperMotor = m_amperMotor;
    }

    @Override
    public void initialize() {
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                shooterMotorSpeed1 = SATConstants.PODIUM.shooter1;
                shooterMotorSpeed2 = SATConstants.PODIUM.shooter2;
                break;
            case SUBWOOFER:
                shooterMotorSpeed1 = SATConstants.SUB.shooter1;
                shooterMotorSpeed2 = SATConstants.SUB.shooter2;
                break;
            case WING:
                shooterMotorSpeed1 = SATConstants.WING.shooter1;
                shooterMotorSpeed2 = SATConstants.WING.shooter2;
                break;
            case AMP:
                shooterMotorSpeed1 = SATConstants.AMP.shooter1;
                shooterMotorSpeed2 = SATConstants.AMP.shooter2;
                break;
            case START:
                shooterMotorSpeed1 = SATConstants.START.shooter1;
                shooterMotorSpeed2 = SATConstants.START.shooter2;
                break;
            default:
                shooterMotorSpeed1 = SATConstants.SUB.shooter1;
                shooterMotorSpeed2 = SATConstants.SUB.shooter2;
                break;
        }

        if (ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AMP){
            m_amperMotor.startAmperMotor();
        }

        // m_index.startIndexMotor();
        m_shooter.setBothShooterMotor(shooterMotorSpeed1, shooterMotorSpeed2);
    }

    @Override
    public boolean isFinished(){
        return (Constants.isWithinTol(shooterMotorSpeed1, m_shooter.getshooterMotorSpeed(), Constants.ShooterConstants.tol)
        &&         
        Constants.isWithinTol(shooterMotorSpeed2, -1 * m_shooter.getshooterMotor1Speed(), Constants.ShooterConstants.tol));

    }

}
