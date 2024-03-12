package frc.robot.subsystems.index;

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
import frc.robot.Constants.IndexConstants;
import frc.robot.Robot;
//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Index extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();
  
  private final TalonFX IndexMotor = new TalonFX(IndexConstants.kIndexMotorId);
  private final PositionVoltage IndexMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final VelocityVoltage IndexMotor_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);


  private double IndexMotorPos;
  private double IndexMotorSpeed;

  /** Creates a new Index. */
  public Index() {
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
      status = IndexMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(IndexMotor, 0.001);
  }



  public void IndexMotorToPosition(double rotations) {
    IndexMotor.setControl(IndexMotor_voltagePosition.withPosition(rotations));
  }

  public double getIndexMotorPosition() {
    return IndexMotorPos;
  }

  public void startIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    // IndexMotor.setControl(IndexMotor_dutyCycleOut.withOutput(-IndexConstants.kIndexMotorSpeed));
    IndexMotor.setControl(IndexMotor_voltageVelocity.withVelocity(IndexConstants.kIndexMotorSpeed));
  }

  public void reverseIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    // IndexMotor.setControl(IndexMotor_dutyCycleOut.withOutput(IndexConstants.kIndexMotorSpeed));
    IndexMotor.setControl(IndexMotor_voltageVelocity.withVelocity(-IndexConstants.kIndexMotorSpeed));
  }


  public void stopIndexMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStop
    IndexMotor.setControl(new NeutralOut());
  }

  public double getIndexMotorSpeed() {
    return IndexMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
  
    IndexMotorPos = IndexMotor.getPosition().getValueAsDouble();
    IndexMotorSpeed = IndexMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putNumber("Index Motor Speed", IndexMotorSpeed);
    SmartDashboard.putNumber("Index Motor Temperature", IndexMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Index rpm", IndexMotor.getVelocity().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_IndexMotor_canbus1signal1 = IndexMotor.getPosition();
    StatusSignal<Double> m_IndexTemp_canbus1signal1 = IndexMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IndexMotor_canbus1signal1);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IndexTemp_canbus1signal1);
    ParentDevice.optimizeBusUtilizationForAll(IndexMotor);
  }
}
