package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.pivot.Pivot;

public class CommandPivotDynamicPose extends Command {
  private final Pivot m_pivot;
  private double pos;
  
  public CommandPivotDynamicPose(Pivot i) {
    this.m_pivot = i;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
    pos = Constants.Vision.pivotEncoderCalculator(Swerve.poseEstimator.getEstimatedPosition());
    m_pivot.leaderGoToPosition(pos);
    SmartDashboard.putNumber("pivot is trying to go to this pos", pos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Constants.isWithinTol(pos, m_pivot.getLeaderPos(), Constants.ArmConstants.tol);
  }
}
