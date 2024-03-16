package frc.robot.commands.ShooterSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;

public class CommandShooterStart extends Command {
  private final Shooter m_shooter;
  private double motor1;
  private double motor2;

  public CommandShooterStart(Shooter i, double motor1, double motor2) {
    m_shooter = i;
    this.motor1 = motor1;
    this.motor2 = motor2;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setBothShooterMotor(motor1, motor2);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(m_shooter.getshooterMotorSpeed() - motor1) <= 1) && (Math.abs(m_shooter.getshooterMotor1Speed() - motor2) <= 1));
  }
}
