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
import frc.robot.Constants.ClimberConstants;

public class Climbers extends SubsystemBase{
    // Hardware
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;
    // private static final DigitalSource throughBore = new DigitalInput(9);
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

    //Check if inverted is applied 
    public static Boolean status1OK = false; 
    public static Boolean status2OK = false;

    private double targetPose;

    public Climbers(){
        leaderTalon = new TalonFX(ClimberConstants.leaderID, "rio");
        followerTalon = new TalonFX(ClimberConstants.followerTalon, "rio");
        followerTalon.setControl(new Follower(ClimberConstants.leaderID, true));

        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;



        // Leader motor configs
        configLeader.Slot0.kP = ClimberConstants.leaderKP;
        configLeader.Slot0.kI = ClimberConstants.leaderKI;
        configLeader.Slot0.kD = ClimberConstants.leaderKD;
        configLeader.Voltage.PeakForwardVoltage = 12;
        configLeader.Voltage.PeakReverseVoltage  = -12;
        configLeader.MotorOutput.Inverted =
        ClimberConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeader.Feedback.RotorToSensorRatio = ClimberConstants.rotorToSensorRatio;
        configLeader.Feedback.SensorToMechanismRatio = ClimberConstants.sensorToMechanismRatio;
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

        StatusCode status1 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 20; ++i) {
            status1 = leaderTalon.getConfigurator().apply(configLeader);
            if (status1.isOK())
                status1OK = true;
                SmartDashboard.putBoolean("Status 1 Climbers", Climbers.status1OK);
                break;
        }
        if (!status1.isOK()) {
            System.out.println("Could not apply configs, error code: " + status1.toString());
        }

        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 20; ++i) {
            status2 = followerTalon.getConfigurator().apply(configLeader);
        if (status2.isOK())
            status1OK = true;
            SmartDashboard.putBoolean("Status 2 Climbers", Climbers.status2OK);
            break;
        }
        if (!status2.isOK()) {
            System.out.println("Could not apply configs, error code: " + status2.toString());
        }

        leaderTalon.setPosition(0);
        followerTalon.setPosition(0);

        // throughBoreEncoder.setDistancePerRotation(targetPose);;
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Leader Motor Temperature", leaderTalon.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Leader Climber Motor Position", leaderTalon.getPosition().getValueAsDouble());


        SmartDashboard.putNumber("Follower Motor Temperature", followerTalon.getDeviceTemp().getValueAsDouble());
       
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

    public void applyConfig(){
        leaderTalon.getConfigurator().apply(configLeader);
        followerTalon.getConfigurator().apply(configLeader);

    }
}
