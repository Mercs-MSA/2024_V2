package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  
    public final XboxController driver_rumble = new XboxController(0); 
    public final XboxController operator_rumble = new XboxController(1);

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
    if (m_beamBreak.detectsNote()) {
      CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
          new CommandRumble(0.8, driver_rumble),
          new CommandRumble(0.8, operator_rumble),
          new WaitCommand(1),
          new CommandRumble(0, driver_rumble),
          new CommandRumble(0, operator_rumble))
      );
    }
  }

  @Override
  public boolean isFinished() {
    boolean hasNote = m_beamBreak.detectsNote();
    return hasNote;
    //Math.abs(m_intake.getIntakeMotorSpeed() - IntakeConstants.kIntakeMotorSpeed) <= IntakeConstants.kIntakeMotorDCTolerance;
  }
}

