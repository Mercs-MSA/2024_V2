package frc.robot.subsystems.sensors;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeamBreakConstants;

public class BeamBreak extends SubsystemBase{
    private final DigitalInput beamBreak = new DigitalInput(BeamBreakConstants.port);
    private boolean detectsNote = false;
    BiConsumer<Boolean, Boolean> callback = (risingEdge, fallingEdge) -> {
        if (risingEdge){
            this.detectsNote = false;
            // RobotContainer.stopEverything();
        }
        if (fallingEdge){
            this.detectsNote = true;
            // RobotContainer.prepShooter();
        }
    };
    private AsynchronousInterrupt asynchronousInterrupt = new AsynchronousInterrupt(beamBreak, callback);

    public BeamBreak(){
        asynchronousInterrupt.disable();
    }

    @Override
    public void periodic() {
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
