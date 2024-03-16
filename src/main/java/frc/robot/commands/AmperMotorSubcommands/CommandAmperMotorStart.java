package frc.robot.commands.AmperMotorSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amperMotor.AmperMotor;

public class CommandAmperMotorStart extends Command {
  private final AmperMotor m_amper;
  
  public CommandAmperMotorStart(AmperMotor i) {
    m_amper = i;
    addRequirements(m_amper);
  }

  @Override
  public void initialize() {
    m_amper.startAmperMotor();
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

