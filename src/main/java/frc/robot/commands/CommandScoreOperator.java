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

public class CommandScoreOperator extends Command {
    private double pivotPos = 0;
    private double shooterMotorSpeed1, shooterMotorSpeed2 = 0;
    private Pivot m_pivot;
    private Amper m_amper;
    private Index m_index;
    private Shooter m_shooter;
    private AmperMotor m_amperMotor;

    public CommandScoreOperator(Pivot m_pivot, Amper m_amper, Index m_index, Shooter m_shooter, AmperMotor m_amperMotor) {
        this.m_pivot = m_pivot;
        this.m_amper = m_amper;
        this.m_index = m_index;
        this.m_shooter = m_shooter;
        this.m_amperMotor = m_amperMotor;
    }

    @Override
    public void initialize() {
        switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                pivotPos = SATConstants.PODIUM.pivot;
                shooterMotorSpeed1 = SATConstants.PODIUM.shooter1;
                shooterMotorSpeed2 = SATConstants.PODIUM.shooter2;
                break;
            case SUBWOOFER:
                pivotPos = SATConstants.SUB.pivot;
                shooterMotorSpeed1 = SATConstants.SUB.shooter1;
                shooterMotorSpeed2 = SATConstants.SUB.shooter2;
                break;
            case WING:
                pivotPos = SATConstants.WING.pivot;
                shooterMotorSpeed1 = SATConstants.WING.shooter1;
                shooterMotorSpeed2 = SATConstants.WING.shooter2;
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

        if (ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AUTOAIM){
            m_pivot.leaderGoToPosition(Constants.Vision.pivotEncoderCalculator(Swerve.poseEstimator.getEstimatedPosition()));
        }
        else {
            m_pivot.leaderGoToPosition(pivotPos);
        }

        if (ScoringConstants.currentScoringMode == ScoringConstants.ScoringMode.AMP){
            m_amper.setServo(0, 0);
            m_amperMotor.startAmperMotor();
        }
        else {
            m_amper.setServo(1, 1);
        }
    }

    @Override
    public boolean isFinished(){
        return Constants.isWithinTol(this.pivotPos, m_pivot.getLeaderPos(), Constants.ArmConstants.tol);
    }

}
