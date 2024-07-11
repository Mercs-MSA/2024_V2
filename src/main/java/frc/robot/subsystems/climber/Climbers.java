package frc.robot.subsystems.climber;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;

public class Climbers {
    //Intialize motors as a leader and a follower
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;


    private final TalonFXConfiguration configLeader = new TalonFXConfiguration();


    private double targetPose;


    public Climbers(){
        leaderTalon = new TalonFX(ClimberConstants.leaderID, "rio");
        followerTalon = new TalonFX(ClimberConstants.followerTalon, "rio");
        followerTalon.setControl(new Follower(ClimberConstants.leaderID, true));

        CANcoderConfiguration climberEncoderConfig = new CANcoderConfiguration();
        climberEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;



        // Leader motor configs
        configLeader.Slot0.kP = ClimberConstants.leaderKP;
        configLeader.Slot0.kI = ClimberConstants.leaderKI;
        configLeader.Slot0.kD = ClimberConstants.leaderKD;
        configLeader.Voltage.PeakForwardVoltage = 16;
        configLeader.Voltage.PeakReverseVoltage  = -16;
        configLeader.MotorOutput.Inverted =
        ClimberConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeader.Feedback.RotorToSensorRatio = ClimberConstants.rotorToSensorRatio;
        configLeader.Feedback.SensorToMechanismRatio = ClimberConstants.sensorToMechanismRatio;


        leaderTalon.getConfigurator().apply(configLeader, 1.0);
        followerTalon.getConfigurator().apply(configLeader, 1.0);


    
}
}
