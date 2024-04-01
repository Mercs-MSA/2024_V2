package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.ApriltagVision;


public class CommandPivotToPose extends Command {
  private final Pivot m_pivot;
  private ApriltagVision m_ApriltagVision;
  private double pivotPos;
  private boolean useStateSystem;
  private double distance;
  
  public CommandPivotToPose(Pivot i, double pos) {
    this.m_pivot = i;
    this.pivotPos = pos;
    this.useStateSystem = false;
    addRequirements(m_pivot);
  }

  public CommandPivotToPose(Pivot i) {
    this.m_pivot = i;
    this.useStateSystem = true;
    addRequirements(m_pivot);
  }

  public CommandPivotToPose(Pivot i, ApriltagVision v) {
    this.m_pivot = i;
    this.useStateSystem = true;
    this.m_ApriltagVision = v;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
    
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
          pivotPos = Constants.Vision.pivotAngleCalculator(m_ApriltagVision.distance);
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

