package frc.robot.commands.BaseSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.SAT.SAT;

public class CommandBaseScoringPosition extends Command {
    double basePos;
    public SAT m_SAT;

    public CommandBaseScoringPosition(SAT s) {
        m_SAT = s;
        addRequirements(m_SAT);
    }

    @Override
    public void initialize() {
        switch (ScoringConstants.currentScoringMode) {
            case AMP:
                basePos = SATConstants.AMP_STAGE_1.motor1_base;
                break;
            case TRAP:
                basePos = SATConstants.TRAP.motor1_base;
                break;
            case SUBWOOFER:
                basePos = SATConstants.SUBWOOFER.motor1_base;
                break;
            case WING:
            case PODIUM:
                basePos = SATConstants.START.motor1_base;
                break;
        }

        m_SAT.moveBaseMotors(basePos); 
        SmartDashboard.putString("base, I'm trying to go here: ", basePos + " ");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interupted) {
        SmartDashboard.putBoolean("is base done?", 
            isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                SATConstants.MOTOR_TOLERANCE
            )
        );
    }

    @Override
    public boolean isFinished() {
        return (
            isWithinTol(
                basePos, 
                m_SAT.getBase1Pos(),
                SATConstants.MOTOR_TOLERANCE
            )
        );
    }

    public boolean isWithinTol(double targetPose, double currentPose, double tolerance) {
        return (Math.abs(targetPose - currentPose) <= tolerance);
    }
}
