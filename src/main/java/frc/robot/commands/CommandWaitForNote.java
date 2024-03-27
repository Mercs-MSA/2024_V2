package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.BeamBreak;

public class CommandWaitForNote extends Command {
    private final BeamBreak m_beamBreak;
  
    public CommandWaitForNote(BeamBreak i) {
        m_beamBreak = i;
        addRequirements(m_beamBreak);
    }

    @Override
    public void initialize() {
        m_beamBreak.enableAsynchronousInterrupt();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        m_beamBreak.disableAsynchronousInterrupt();
    }

    @Override
    public boolean isFinished() {
        return m_beamBreak.detectsNote() == true;
    }
}