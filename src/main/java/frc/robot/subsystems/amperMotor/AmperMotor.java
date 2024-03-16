package frc.robot.subsystems.amperMotor;

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
import frc.robot.Constants.AmperConstants;
import frc.robot.Robot;
//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class AmperMotor extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();
  
  private final TalonFX AmperMotor = new TalonFX(AmperConstants.kAmperMotorId);
  private final PositionVoltage AmperMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final VelocityVoltage AmperMotor_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);


  private double AmperMotorPos;
  private double AmperMotorSpeed;

  /** Creates a new Index. */
  public AmperMotor() {
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
      status = AmperMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(AmperMotor, 0.001);
  }



  public void AmperMotorToPosition(double rotations) {
    AmperMotor.setControl(AmperMotor_voltagePosition.withPosition(rotations));
  }

  public double getAmperMotorPosition() {
    return AmperMotorPos;
  }

  public void startAmperMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    // AmperMotor.setControl(AmperMotor_dutyCycleOut.withOutput(-IndexConstants.kAmperMotorSpeed));
    AmperMotor.setControl(AmperMotor_voltageVelocity.withVelocity(-AmperConstants.kAmperMotorSpeed));
  }

  public void reverseAmperMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStart
    // AmperMotor.setControl(AmperMotor_dutyCycleOut.withOutput(IndexConstants.kAmperMotorSpeed));
    AmperMotor.setControl(AmperMotor_voltageVelocity.withVelocity(AmperConstants.kAmperMotorSpeed));
  }


  public void stopAmperMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIndexStop
    AmperMotor.setControl(new NeutralOut());
  }

  public double getAmperMotorSpeed() {
    return AmperMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
  
    AmperMotorPos = AmperMotor.getPosition().getValueAsDouble();
    AmperMotorSpeed = AmperMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putNumber("Index Motor Speed", AmperMotorSpeed);
    SmartDashboard.putNumber("Index Motor Temperature", AmperMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Index rpm", AmperMotor.getVelocity().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_AmperMotor_canbus1signal1 = AmperMotor.getPosition();
    StatusSignal<Double> m_IndexTemp_canbus1signal1 = AmperMotor.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_AmperMotor_canbus1signal1);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IndexTemp_canbus1signal1);
    ParentDevice.optimizeBusUtilizationForAll(AmperMotor);
  }
}
