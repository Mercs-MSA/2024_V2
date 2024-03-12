package frc.robot.subsystems.arm;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    // Hardware
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;
    private CANcoder absoluteEncoder;

    // Status Signals
    private final StatusSignal<Double> internalPositionRotations;
    private final StatusSignal<Double> encoderAbsolutePositionRotations;
    private final StatusSignal<Double> encoderRelativePositionRotations;
    private final StatusSignal<Double> velocityRps;
    private final List<StatusSignal<Double>> appliedVoltage;
    private final List<StatusSignal<Double>> supplyCurrent;
    private final List<StatusSignal<Double>> tempCelsius;
    private final PositionVoltage leaderVoltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // Config
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public Arm(){
        leaderTalon = new TalonFX(ArmConstants.leaderID);
        followerTalon = new TalonFX(ArmConstants.followerTalon);
        followerTalon.setControl(new Follower(ArmConstants.leaderID, true));
        absoluteEncoder = new CANcoder(ArmConstants.armEncoderID);

        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1.0);

        // Leader motor configs
        config.Slot0.kP = ArmConstants.leaderKP;
        config.Slot0.kI = ArmConstants.leaderKI;
        config.Slot0.kD = ArmConstants.leaderKD;
        config.Voltage.PeakForwardVoltage = 16;
        config.Voltage.PeakReverseVoltage  = -16;
        config.MotorOutput.Inverted =
        ArmConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackRemoteSensorID = ArmConstants.armEncoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.RotorToSensorRatio = ArmConstants.rotorToSensorRatio;
        config.Feedback.SensorToMechanismRatio = ArmConstants.sensorToMechanismRatio;
        leaderTalon.getConfigurator().apply(config, 1.0);

        // Status signals
        internalPositionRotations = leaderTalon.getPosition();
        encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
        encoderRelativePositionRotations = absoluteEncoder.getPosition();
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

        BaseStatusSignal.setUpdateFrequencyForAll(
        1000, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

    }

    public void setBrakeMode(boolean enabled){
        leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    // This is for test purposes only
    public void leaderGoToPositionIncrement(double increment) {
        double targetPose = leaderTalon.getPosition().getValueAsDouble() + (increment);
        leaderTalon.setControl(leaderVoltagePosition.withPosition(targetPose));
    }

    public void leaderGoToPosition(double pos) {
        leaderTalon.setControl(leaderVoltagePosition.withPosition(pos));
    }

    public void stop(){
        leaderTalon.setControl(new NeutralOut());
    }
}
