package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();
  
  private final TalonFX shooterMotor = new TalonFX(ShooterConstants.kshooterMotorId);
  private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.kshooterMotor1Id);
  private final PositionVoltage shooterMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final VelocityVoltage shooterMotor_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);


  private double shooterMotorPos;
  private double shooterMotorSpeed;

  /** Creates a new Index. */
  public Shooter() {
    SmartDashboard.putString("sensor debug", "init");

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.Slot0.kP = 25.0; // An error of 0.5 rotations results in 1.2 volts output
    configs.Slot0.kD = 0.4; // A change of 1 rotation per second results in 0.1 volts output

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 16;
    configs.Voltage.PeakReverseVoltage = -16;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shooterMotor1.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(shooterMotor, 0.001);
  }



  public void shooterMotorToPosition(double rotations) {
    shooterMotor.setControl(shooterMotor_voltagePosition.withPosition(rotations));
  }

  public double getshooterMotorPosition() {
    return shooterMotorPos;
  }

  public void setBothShooterMotor(double speed, double speed1) {
    shooterMotor.setControl(shooterMotor_voltageVelocity.withVelocity(speed));
    shooterMotor1.setControl(shooterMotor_voltageVelocity.withVelocity(speed1));
  }


  public void stopshooterMotor() {
    shooterMotor.setControl(new NeutralOut());
    shooterMotor1.setControl(new NeutralOut());
  }

  public double getshooterMotorSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
  
    SmartDashboard.putNumber("Shooter Motor Speed", shooterMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Motor Temperature", shooterMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter rpm", shooterMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("Shooter1 Motor Speed", shooterMotor1.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Shooter1 Motor Temperature", shooterMotor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter1 rpm", shooterMotor1.getVelocity().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_shooterMotor_canbus1signal1 = shooterMotor.getPosition();
    StatusSignal<Double> m_IndexTemp_canbus1signal1 = shooterMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_shooterMotor_canbus1signal1);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IndexTemp_canbus1signal1);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
  }
}
