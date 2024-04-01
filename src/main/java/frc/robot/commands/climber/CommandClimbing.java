package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SATConstants;
import frc.robot.Constants.SATConstants.START;
import frc.robot.Constants.ScoringConstants;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.climber.climber.climberStates;
import frc.robot.Constants.climberConstants;


public class CommandClimbing extends Command {
  private final climber m_climber;
  private double climberRightPos;
  private double climberLeftPos;

  private boolean useStateSystem;

  
  public CommandClimbing(climber m_climber) {
    this.m_climber = m_climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {    
    // SmartDashboard.putNumber("pivot is trying to go to this pos", pivotPos);

    if (useStateSystem){
      switch (climberConstants.climberMode) {

        case UP:
          climberRightPos = climberConstants.RIGHT_TOP_POSITION;
          climberLeftPos = climberConstants.LEFT_TOP_POSITION;
          break;

        case DOWN:
          climberRightPos = climberConstants.RIGHT_BOTTOM_POSITION;
          climberLeftPos = climberConstants.LEFT_BOTTOM_POSITION;
          break;

        case NONE:
          break;

        case ERROR:
          break;

        default:
          break;
      }
    }

    m_climber.climb(climberRightPos, climberLeftPos);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}


