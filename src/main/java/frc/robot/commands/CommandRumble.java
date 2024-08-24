package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandRumble extends Command {


    private final double m_power;
    private final XboxController m_controller;
  
  
    public CommandRumble(double power, XboxController controller) {
        m_power = power;
        m_controller = controller;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("power", m_power);
        // while (Timer.getFPGATimestamp() - initTime <= StartTime) { 
        //     m_controller.setRumble(RumbleType.kBothRumble, m_power); }
        // m_controller.setRumble(RumbleType.kBothRumble, 0);
        m_controller.setRumble(RumbleType.kBothRumble, m_power);
       
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
