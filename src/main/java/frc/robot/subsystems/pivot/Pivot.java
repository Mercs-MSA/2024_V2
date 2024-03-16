package frc.robot.subsystems.pivot;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ScoringConstants;

public class Pivot extends SubsystemBase{
    // Hardware
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;
    private static final DigitalSource throughBore = new DigitalInput(9);
    // DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(throughBore);
    // private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;

    // Status Signals
    private final StatusSignal<Double> internalPositionRotations;
    private final StatusSignal<Double> velocityRps;
    private final List<StatusSignal<Double>> appliedVoltage;
    private final List<StatusSignal<Double>> supplyCurrent;
    private final List<StatusSignal<Double>> tempCelsius;
    private final PositionVoltage leaderVoltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // Config
    private final TalonFXConfiguration configLeader = new TalonFXConfiguration();

    private double targetPose;

    public Pivot(){
        leaderTalon = new TalonFX(ArmConstants.leaderID);
        followerTalon = new TalonFX(ArmConstants.followerTalon);
        followerTalon.setControl(new Follower(ArmConstants.leaderID, true));

        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Leader motor configs
        configLeader.Slot0.kP = ArmConstants.leaderKP;
        configLeader.Slot0.kI = ArmConstants.leaderKI;
        configLeader.Slot0.kD = ArmConstants.leaderKD;
        configLeader.Voltage.PeakForwardVoltage = 16;
        configLeader.Voltage.PeakReverseVoltage  = -16;
        configLeader.MotorOutput.Inverted =
        ArmConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeader.Feedback.RotorToSensorRatio = ArmConstants.rotorToSensorRatio;
        configLeader.Feedback.SensorToMechanismRatio = ArmConstants.sensorToMechanismRatio;
        leaderTalon.getConfigurator().apply(configLeader, 1.0);
        followerTalon.getConfigurator().apply(configLeader, 1.0);

        // Status signals
        internalPositionRotations = leaderTalon.getPosition();
        velocityRps = leaderTalon.getVelocity();
        appliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
        supplyCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
        tempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());
        BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

        leaderTalon.setPosition(0);
        followerTalon.setPosition(0);

        // throughBoreEncoder.setDistancePerRotation(targetPose);;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Leader Motor Temperature", leaderTalon.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Leader Motor Position", leaderTalon.getPosition().getValueAsDouble());


        SmartDashboard.putNumber("Follower Motor Temperature", followerTalon.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Follower Motor Position", followerTalon.getPosition().getValueAsDouble());
    }

    public void setBrakeMode(boolean enabled){
        leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public double getLeaderPos(){
        return leaderTalon.getPosition().getValueAsDouble();
    }

    public void leaderGoToPositionIncrement(double increment) {
        targetPose = targetPose + (increment*2);
        leaderTalon.setControl(leaderVoltagePosition.withPosition(targetPose));
    }

    public void leaderGoToPosition(double pos) {
        leaderTalon.setControl(leaderVoltagePosition.withPosition(pos));
    }

    public void stop(){
        leaderTalon.setControl(new NeutralOut());
    }
}
