package frc.robot.commands.AmperSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amper.Amper;

public class CommandAmperScoreAmp extends Command {
  private final Amper m_amper;
  
  public CommandAmperScoreAmp(Amper i) {
    this.m_amper = i;
    addRequirements(m_amper);
  }

  @Override
  public void initialize() {
    m_amper.setServo(0, 0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
    //return Math.abs(m_index.getIndexMotorSpeed() + IntakeConstants.kIndexMotorSpeed) <= IntakeConstants.kIndexMotorDCTolerance;
  }
}

