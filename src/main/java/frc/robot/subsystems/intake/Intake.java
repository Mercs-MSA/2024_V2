package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
//  USE NEXT LINE FOR TESTING
import frc.robot.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  // USE NEXT LINE FOR TESTING
  public boolean simulationDebugMode = Robot.isSimulation();
  
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId);
  private final PositionVoltage intakeMotor_voltagePosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  private final VelocityVoltage intakeMotor_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);


  private double intakeMotorPos;
  private double intakeMotorSpeed;

  /** Creates a new intake. */
  public Intake() {
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
      status = intakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    optimization_for_CAN();

    // USE NEXT LINE FOR TESTING
    PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.001);
  }



  public void intakeMotorToPosition(double rotations) {
    intakeMotor.setControl(intakeMotor_voltagePosition.withPosition(rotations));
  }

  public double getIntakeMotorPosition() {
    return intakeMotorPos;
  }

  public void startIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart
    // intakeMotor.setControl(intakeMotor_dutyCycleOut.withOutput(-IntakeConstants.kIntakeMotorSpeed));
    intakeMotor.setControl(intakeMotor_voltageVelocity.withVelocity(IntakeConstants.kIntakeMotorSpeed));
  }

  public void reverseIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStart
    // intakeMotor.setControl(intakeMotor_dutyCycleOut.withOutput(IntakeConstants.kIntakeMotorSpeed));
    intakeMotor.setControl(intakeMotor_voltageVelocity.withVelocity(-IntakeConstants.kIntakeMotorSpeed));
  }


  public void stopIntakeMotor() {
    // WARNING!! 
    // DO NOT USE THIS FUNCTION DIRECTLY!!
    // INSTEAD USE: CommandOverrideIntakeStop
    intakeMotor.setControl(new NeutralOut());
  }

  public double getIntakeMotorSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
  
    intakeMotorPos = intakeMotor.getPosition().getValueAsDouble();
    intakeMotorSpeed = intakeMotor.getDutyCycle().getValueAsDouble();

    SmartDashboard.putNumber("Intake Motor Speed", intakeMotorSpeed);
    SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("intake rpm", intakeMotor.getVelocity().getValueAsDouble());
  }
  
  // USE FOR TESTING ALSO
  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  public void optimization_for_CAN() {
    StatusSignal<Double> m_IntakeMotor_canbus1signal1 = intakeMotor.getPosition();
    StatusSignal<Double> m_IntakeTemp_canbus1signal1 = intakeMotor.getDeviceTemp();
    StatusSignal<Double> m_IntakeDutyCycle_canbus1signal1 = intakeMotor.getDutyCycle();
    BaseStatusSignal.setUpdateFrequencyForAll(60, m_IntakeMotor_canbus1signal1);
    BaseStatusSignal.setUpdateFrequencyForAll(1, m_IntakeTemp_canbus1signal1);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor);
  }
}
