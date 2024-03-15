package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;

public class CommandPivotToPose extends Command {
  private final Pivot m_pivot;
  private double pos;
  
  public CommandPivotToPose(Pivot i, double pos) {
    this.m_pivot = i;
    this.pos = pos;
    addRequirements(m_pivot);
  }

  @Override
  public void initialize() {
    m_pivot.leaderGoToPosition(pos);
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

