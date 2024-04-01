package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;

public class CommandIndexStopNeutral extends Command {
  private final Index m_index;
  
  public CommandIndexStopNeutral(Index i) {
    m_index = i;
    addRequirements(m_index);
  }

  @Override
  public void initialize() {
    m_index.stopIndexMotor();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // return m_index.getIndexMotorSpeed() == 0;
    return true;
  }
}

