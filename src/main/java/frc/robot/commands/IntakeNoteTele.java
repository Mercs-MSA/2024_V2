package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.amper.Amper;
import frc.robot.subsystems.amperMotor.AmperMotor;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.sensors.BeamBreak;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeNoteTele extends Command {
  private final Intake m_intake;
  private final Index m_index;
  private final BeamBreak m_beamBreak;
  
  public IntakeNoteTele(Intake intake, Index index, BeamBreak beamBreak) {
    m_intake = intake;
    m_index = index;
    m_beamBreak = beamBreak;
    //addRequirements(m_intake, m_index, m_beamBreak);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Intake done", false); 
          m_intake.startIntakeMotor();
          m_index.startIndexMotor();

      



      
      }

    
  

  @Override
  public void execute() {


  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Intake done", true);
    m_intake.stopIntakeMotor();
    m_index.stopIndexMotor();
  }

  @Override
  public boolean isFinished() {
    boolean hasNote = m_beamBreak.detectsNote();
    return hasNote;
    //Math.abs(m_intake.getIntakeMotorSpeed() - IntakeConstants.kIntakeMotorSpeed) <= IntakeConstants.kIntakeMotorDCTolerance;
  }
}

