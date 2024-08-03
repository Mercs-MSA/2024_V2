package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.ApriltagVision;


public class CommandPivotConstantAim extends Command {
  private final Pivot m_pivot;
  private ApriltagVision m_ApriltagVision;
  private double pivotPos;
  private boolean useStateSystem;
  private double distance;
  
  public CommandPivotConstantAim(Pivot i, double pos) {
    this.m_pivot = i;
    this.pivotPos = pos;
    this.useStateSystem = false;
    addRequirements(m_pivot);
  }

  public CommandPivotConstantAim(Pivot i) {
    this.m_pivot = i;
    this.useStateSystem = true;
    addRequirements(m_pivot);
  }

  public CommandPivotConstantAim(Pivot i, ApriltagVision v) {
    this.m_pivot = i;
    this.useStateSystem = true;
    this.m_ApriltagVision = v;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
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
          pivotPos = Constants.Vision.pivotInterpolationPosCalculator();
          break;
        case NEW_SHUNT:
          pivotPos = SATConstants.NEW_SHUNT.pivot;
          break;
        default:
          break;
      }
    }
    if (pivotPos > 0){
    m_pivot.leaderGoToPosition(pivotPos);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;}
}

