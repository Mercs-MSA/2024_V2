package frc.robot.commands.ShooterSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooterStopNeutral extends Command {
  private final Shooter m_shooter;

  public CommandShooterStopNeutral(Shooter i) {
    m_shooter = i;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopshooterMotor();
  }

  @Override
  public boolean isFinished() {
    return true;
    //Math.abs(m_intake.getIntakeMotorSpeed() - IntakeConstants.kIntakeMotorSpeed) <= IntakeConstants.kIntakeMotorDCTolerance;
  }
}

