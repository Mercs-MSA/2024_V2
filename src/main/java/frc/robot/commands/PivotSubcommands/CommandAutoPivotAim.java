package frc.robot.commands.PivotSubcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;


public class CommandAutoPivotAim extends Command{
    private final Pivot m_pivot;
    private double pivotPos;
    
    
    
    public CommandAutoPivotAim(Pivot i) {
        m_pivot = i;
        addRequirements(m_pivot);
  }
  
    @Override
    public void initialize() {
        pivotPos = Constants.Vision.pivotEncoderCalculator();
        if (pivotPos > 0) {
            m_pivot.leaderGoToPosition(pivotPos);
        }
        
  }
  
    @Override
    public void execute() {}
  
    @Override
    public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Constants.isWithinTol(pivotPos, m_pivot.getLeaderPos(), Constants.ArmConstants.tol);
  }

}
         



            
