package frc.robot.commands.IndexSubcommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.sensors.BeamBreak;



public class CommandIndexStartAuto extends Command {
  private final Index m_index;
  private final BeamBreak m_beamBreak;
  boolean has_seen = false;
  
  public CommandIndexStartAuto(Index index, BeamBreak beamBreak) {
    m_index = index;
    m_beamBreak = beamBreak;
    
    addRequirements(m_index);
  }

  @Override
  public void initialize() {

    has_seen = true;
    SmartDashboard.putBoolean("has seen note", has_seen);
    m_index.startIndexMotor(); 
  }


  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (has_seen = true) && (!m_beamBreak.detectsNote());
    //return Math.abs(m_index.getIndexMotorSpeed() + IntakeConstants.kIndexMotorSpeed) <= IntakeConstants.kIndexMotorDCTolerance;
  }
}

