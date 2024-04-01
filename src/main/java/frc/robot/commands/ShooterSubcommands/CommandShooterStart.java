package frc.robot.commands.ShooterSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.ScoringConstants;
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

  public CommandShooterStart(Shooter i) {
    m_shooter = i;
      switch (ScoringConstants.currentScoringMode) {
            case PODIUM:
                motor1 = SATConstants.PODIUM.shooter1;
                motor2 = SATConstants.PODIUM.shooter2;
                break;
            case SUBWOOFER:
                motor1 = SATConstants.SUB.shooter1;
                motor2 = SATConstants.SUB.shooter2;
                break;
            case WING:
                motor1 = SATConstants.WING.shooter1;
                motor2 = SATConstants.WING.shooter2;
                break;
            case AMP:
                motor1 = SATConstants.AMP.shooter1;
                motor2 = SATConstants.AMP.shooter2;
                break;
            case START:
                motor1 = SATConstants.START.shooter1;
                motor2 = SATConstants.START.shooter2;
                break;
            default:
                motor1 = SATConstants.SUB.shooter1;
                motor2 = SATConstants.SUB.shooter2;
                break;
        }
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
    return ((Math.abs(m_shooter.getshooterMotorSpeed() - motor1) <= 1.3) || (Math.abs(m_shooter.getshooterMotor1Speed() - motor2) <= 1.3));
  }
}

