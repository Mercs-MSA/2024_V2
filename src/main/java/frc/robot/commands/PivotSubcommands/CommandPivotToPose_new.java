package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.pivot.Pivot;


public class CommandPivotToPose_new extends Command {
  private final Pivot m_pivot;
  private final CommandSwerveDrivetrain swerve;
  private double pivotPos;
  private boolean useStateSystem;
  
  public CommandPivotToPose_new(Pivot i, double pos, CommandSwerveDrivetrain s) {
    this.m_pivot = i;
    this.pivotPos = pos;
    this.useStateSystem = false;
    this.swerve = s;
    addRequirements(m_pivot);
  }

  public CommandPivotToPose_new(Pivot i, CommandSwerveDrivetrain s) {
    this.m_pivot = i;
    this.useStateSystem = true;
    this.swerve = s;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
    var currPose = swerve.getState().Pose;
    
    SmartDashboard.putNumber("pivot is trying to go to this pos", pivotPos);

    if (useStateSystem){
      switch (ScoringConstants.currentScoringMode) {
        case PODIUM:
          pivotPos = SATConstants.PODIUM.pivot;
          break;
        case SUBWOOFER:
          pivotPos = SATConstants.SUB.pivot;
          break;
        case WING:
          pivotPos = SATConstants.WING.pivot;
          break;
        case AMP:
          pivotPos = SATConstants.AMP.pivot;
          break;
        case START:
          pivotPos = SATConstants.START.pivot;
          break;
        case AUTOAIM:
          pivotPos = Constants.Vision.pivotEncoderCalculator();
          break;
        default:
          break;
      }
    }
    m_pivot.leaderGoToPosition(pivotPos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Constants.isWithinTol(pivotPos, m_pivot.getLeaderPos(), Constants.ArmConstants.tol);
  }
}