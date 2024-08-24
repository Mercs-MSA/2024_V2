package frc.robot.subsystems.sensors;

import java.util.function.BiConsumer;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.BeamBreakConstants;
import frc.robot.commands.CommandRumble;

public class BeamBreak extends SubsystemBase{
    private final DigitalInput beamBreak = new DigitalInput(BeamBreakConstants.port);
    private boolean detectsNote = false;
    
    BiConsumer<Boolean, Boolean> callback = (risingEdge, fallingEdge) -> {
        SmartDashboard.putBoolean("hasFall", fallingEdge);
        SmartDashboard.putBoolean("hasRise", risingEdge);
        if (risingEdge){
            this.detectsNote = false;
            // RobotContainer.stopEverything();
        }
        if (fallingEdge){
            this.detectsNote = true;
            RobotContainer.stopEverything().schedule();
            // RobotContainer.prepShooter();
        }
        // RobotContainer.stopEverything();
    };
    private AsynchronousInterrupt asynchronousInterrupt = new AsynchronousInterrupt(beamBreak, callback);

    public BeamBreak(){
        asynchronousInterrupt.enable();
    }

    @Override
    public void periodic() {
        this.detectsNote = !beamBreak.get();
        SmartDashboard.putBoolean("Beam Break Detects Note", beamBreak.get());
    }

    public void enableAsynchronousInterrupt(){
        asynchronousInterrupt.enable();
    }
    
    public void disableAsynchronousInterrupt(){
        asynchronousInterrupt.disable();
    }

    public boolean detectsNote(){
        return this.detectsNote;
    }


}
