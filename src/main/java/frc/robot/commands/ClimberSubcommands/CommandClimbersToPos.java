
package frc.robot.commands.ClimberSubcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.climber.Climbers;
import frc.robot.subsystems.vision.ApriltagVision;


public class CommandClimbersToPos extends Command {
  private final Climbers m_climbers;
  private double climberPos;
  
  public CommandClimbersToPos(Climbers i, double pos) {
    this.m_climbers = i;
    this.climberPos = pos;
    addRequirements(m_climbers);
  }



  @Override
  public void initialize() {
    
    SmartDashboard.putNumber("climbers are trying to go to this pos", climberPos);

    
    
    m_climbers.leaderGoToPosition(climberPos);
    
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Constants.isWithinTol(climberPos, m_climbers.getLeaderPos(), Constants.ClimberConstants.tol);
  }
}